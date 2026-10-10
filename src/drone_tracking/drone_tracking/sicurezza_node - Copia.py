#!/usr/bin/env python3
"""Limiti operativi: cosa il velivolo non deve fare, qualunque cosa chieda la missione.

Fino a ora il progetto non aveva alcuna rete: nessun confine, nessun tetto di
quota, nessuna sorveglianza della batteria. Un bersaglio che fugge verso
l'orizzonte veniva inseguito finche c'era corrente, e l'unico comportamento
definito alla morte della percezione era il comando azzerato — che significa
«resta li», ragionevole ma non sufficiente.

Questo nodo e volutamente separato dalla missione e la scavalca. Una rete di
sicurezza che vive dentro la logica che deve sorvegliare non e una rete: se
mission_node si blocca in uno stato imprevisto, deve esserci qualcosa di
esterno che se ne accorge.

Il meccanismo con cui scavalca e il cambio di modo di volo: fuori da GUIDED
ArduPilot ignora i setpoint di velocita, quindi l'intervento vince senza dover
convincere nessun altro nodo a smettere. Il blocco viene comunque pubblicato,
cosi controllo e missione smettono di spingere invece di litigare con l'RTL.

I limiti sono scelte operative e non costanti fisiche, e stanno fra i parametri:
  raggio_max    fin dove si accetta di inseguire, dal punto di decollo
  quota_max     tetto: 120 m e il limite della categoria aperta europea
  quota_min     sotto questa quota la conversione in metri non ha piu senso
  batteria_min  frazione di carica sotto la quale si rientra
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String

from drone_tracking.parametri import parametro  # type: ignore


class SicurezzaNode(Node):
    def __init__(self):
        super().__init__('sicurezza_node')

        qos_mavros = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self.on_posa, qos_mavros)
        self.create_subscription(BatteryState, '/mavros/battery',
                                 self.on_batteria, qos_mavros)
        self.create_subscription(State, '/mavros/state', self.on_stato_volo,
                                 qos_mavros)
        self.create_subscription(Bool, '/sicurezza/reset', self.on_reset, 10)

        self.pub_blocco = self.create_publisher(Bool, '/sicurezza/blocco', 10)
        self.pub_stato = self.create_publisher(String, '/sicurezza/stato', 10)
        self.cli_modo = self.create_client(SetMode, '/mavros/set_mode')

        self.raggio_max = parametro(self, 'raggio_max', 500.0)
        self.quota_max = parametro(self, 'quota_max', 120.0)
        self.quota_min = parametro(self, 'quota_min', 5.0)
        self.batteria_min = parametro(self, 'batteria_min', 0.25)
        # Margine di rientro: senza, appena il velivolo torna dentro il confine
        # il blocco cadrebbe, la missione riprenderebbe a inseguire e lo
        # riporterebbe fuori. Il blocco si sgancia solo con un reset esplicito.
        self.azione = parametro(self, 'azione', 'RTL')
        self.interviene = parametro(self, 'interviene', True)
        self.frequenza_hz = parametro(self, 'frequenza_hz', 2.0)

        self.posizione = None
        self.quota = None
        self.carica = None
        self.modo_volo = ''
        # Il limite inferiore di quota si arma solo dopo che la quota operativa
        # e stata raggiunta: durante il decollo il velivolo e sotto il minimo
        # per forza, e intervenire li significa abortire la salita.
        self.in_quota_operativa = False
        self.bloccato = False
        self.motivo = ''
        self.modo_richiesto = False

        self.create_timer(1.0 / self.frequenza_hz, self.controlla)
        self.get_logger().info(
            'SicurezzaNode avviato — raggio {:.0f} m, quota {:.0f}-{:.0f} m, '
            'batteria {:.0f}%'.format(self.raggio_max, self.quota_min,
                                      self.quota_max, 100 * self.batteria_min))

    # ------------------------------------------------------------- ingressi
    def on_posa(self, msg: PoseStamped):
        p = msg.pose.position
        self.posizione = (p.x, p.y)
        self.quota = p.z

    def on_stato_volo(self, msg: State):
        self.modo_volo = msg.mode

    def on_batteria(self, msg: BatteryState):
        # MAVROS riporta -1 quando la carica non e nota: non e uno zero, ed
        # e la differenza fra «batteria scarica» e «batteria non misurata».
        self.carica = msg.percentage if msg.percentage >= 0.0 else None

    def on_reset(self, msg: Bool):
        if msg.data and self.bloccato:
            self.bloccato = False
            self.motivo = ''
            self.modo_richiesto = False
            self.get_logger().warn('Blocco di sicurezza rimosso su richiesta')

    # -------------------------------------------------------------- verifica
    def _violazione(self):
        """Primo limite superato, o None. L'ordine e per gravita."""
        if self.carica is not None and self.carica < self.batteria_min:
            return 'batteria al {:.0f}%, sotto il minimo del {:.0f}%'.format(
                100 * self.carica, 100 * self.batteria_min)
        if self.quota is not None:
            if self.quota > self.quota_max:
                return 'quota {:.0f} m oltre il tetto di {:.0f}'.format(
                    self.quota, self.quota_max)
            if self.quota > self.quota_min * 1.2:
                self.in_quota_operativa = True
            # Il minimo vale solo dopo aver raggiunto la quota operativa e solo
            # mentre vola la missione: durante il decollo, e durante qualunque
            # discesa comandata, essere sotto il minimo e normale.
            if (self.in_quota_operativa and self.modo_volo == 'GUIDED'
                    and self.quota < self.quota_min):
                return 'quota {:.1f} m sotto il minimo di {:.0f}'.format(
                    self.quota, self.quota_min)
        if self.posizione is not None:
            distanza = math.hypot(*self.posizione)
            if distanza > self.raggio_max:
                return 'distanza {:.0f} m oltre il confine di {:.0f}'.format(
                    distanza, self.raggio_max)
        return None

    def controlla(self):
        violazione = self._violazione()
        if violazione and not self.bloccato:
            self.bloccato = True
            self.motivo = violazione
            self.get_logger().error(
                'LIMITE SUPERATO: {} — inseguimento interrotto'.format(violazione))
            if self.interviene:
                self._cambia_modo()

        self.pub_blocco.publish(Bool(data=self.bloccato))
        self.pub_stato.publish(String(
            data=self.motivo if self.bloccato else 'entro i limiti'))

    def _cambia_modo(self):
        """Chiede il cambio di modo una volta sola, senza bloccare il timer."""
        if self.modo_richiesto:
            return
        if not self.cli_modo.service_is_ready():
            self.get_logger().error(
                'Servizio set_mode non disponibile: il blocco resta ma il '
                'velivolo non e stato comandato')
            return
        self.modo_richiesto = True
        richiesta = SetMode.Request()
        richiesta.custom_mode = self.azione
        futuro = self.cli_modo.call_async(richiesta)
        futuro.add_done_callback(self._esito_modo)

    def _esito_modo(self, futuro):
        try:
            ok = futuro.result().mode_sent
        except Exception as errore:                     # noqa: BLE001
            self.get_logger().error('set_mode fallito: {}'.format(errore))
            return
        if ok:
            self.get_logger().warn('Modo {} comandato'.format(self.azione))
        else:
            # Un rifiuto va detto: RTL e LOITER richiedono una posizione
            # valida, e sotto negazione GNSS possono non essere disponibili.
            self.get_logger().error(
                'Modo {} RIFIUTATO dall autopilota'.format(self.azione))


def main(args=None):
    rclpy.init(args=args)
    nodo = SicurezzaNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
