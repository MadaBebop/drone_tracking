#!/usr/bin/env python3
"""Attacco al GNSS del velivolo, iniettato nel simulatore.

Nodo distinto da jammer_node, perche i due guasti sono fisicamente diversi:
jammer_node disturba il canale con cui il bersaglio viene rilevato, questo
attacca il ricevitore satellitare del drone. Confonderli era il difetto
principale della prima versione del progetto, in cui il topic /gps/jammed
veniva pubblicato ma non guidava nulla.

Il disturbo non viene simulato a livello di topic ROS: viene iniettato nei
parametri del SITL, cosi ad essere messo alla prova e il sistema reale
(autopilota compreso) e non una sua imitazione. Modi disponibili:

  jamming     SIM_GPS1_JAM = 1. Il ricevitore perde e riacquisisce il fix in
              modo intermittente, con l'accuratezza dichiarata che degrada:
              misurato fix_type da 6 a 1, satelliti da 10 a 3, accuratezza
              orizzontale fino a 191 m.
  negazione   SIM_GPS1_ENABLE = 0. Il ricevitore tace del tutto: e il caso
              piu severo, equivalente a un'antenna staccata.
  spoofing    SIM_GPS1_GLTCH_X/Y. Il fix viene falsificato di un offset in
              gradi. Verificato sul topic del fix grezzo: la posizione
              riportata si sposta davvero.

Cosa aspettarsi, misurato prima di scrivere questo nodo. Nessuno dei tre modi
degrada in modo osservabile la stima di posizione dell'autopilota entro il
minuto: lo scarto fra stima dell'EKF e verita a terra di Gazebo resta sotto il
metro e mezzo, come con il GPS sano. Le ragioni sono tre e appartengono
all'autopilota, non a questo progetto: la quota viene dal barometro
(EK3_SRC1_POSZ = 1), la navigazione inerziale non deriva in modo apprezzabile
su tempi brevi, e l'EKF rifiuta un fix incoerente invece di seguirlo.

Il nodo serve percio a dimostrare un'affermazione diversa e piu solida:
l'inseguimento visivo non dipende dal GNSS. Si esegue la stessa missione con
GPS sano e con GPS sotto attacco, e si confrontano durata dell'aggancio,
distanza mediana e frazione di fotogrammi con bersaglio. Se coincidono,
l'indipendenza e provata.
"""
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType, ParameterValue
from std_msgs.msg import Bool, String
from mavros_msgs.srv import ParamSetV2

from drone_tracking.mission_node import FaseMissione  # type: ignore
from drone_tracking.parametri import parametro  # type: ignore

# Parametro del SITL e valori (attacco, riposo) per ciascun modo.
MODI = {
    'jamming':   ('SIM_GPS1_JAM', 1, 0),
    'negazione': ('SIM_GPS1_ENABLE', 0, 1),
}


class GnssDenialNode(Node):
    def __init__(self):
        super().__init__('gnss_denial_node')

        self.modo = str(parametro(self, 'modo', 'jamming'))
        # Ciclo lento di proposito: molto piu lento di quello di jammer_node,
        # per lasciare all'EKF il tempo di derivare in modo osservabile e per
        # assorbire la latenza della scrittura del parametro.
        parametro(self, 'durata_attivo_s', 20.0)
        parametro(self, 'durata_inattivo_s', 40.0)
        # Per il confronto A/B conviene invece l'attacco continuo: si misura la
        # missione intera in condizioni di GNSS compromesso.
        parametro(self, 'sempre_attivo', False)
        # Offset di falsificazione in gradi, usato solo dal modo spoofing.
        # 0.0002 gradi valgono circa 22 metri.
        parametro(self, 'spoofing_gradi', 0.0002)
        # ArduPilot alla perdita del GPS passa in LAND da solo (FS_EKF_ACTION 1).
        # E un comportamento legittimo, ma scavalca mission_node prima che si
        # possa osservare se il controllo visivo tiene la posizione relativa.
        # Si disattiva qui, non in sitl-defaults.parm, per non alterare tutte le
        # altre prove: il ripristino avviene alla chiusura del nodo.
        parametro(self, 'disattiva_failsafe_ekf', True)

        self.attivo = False
        self.fase_missione = FaseMissione.ATTESA.value
        self.istante_cambio = None
        self.failsafe_salvato = None
        self.chiamate_in_corso = []

        self.pub = self.create_publisher(Bool, '/gps/denial_active', 10)
        self.create_subscription(String, '/mission/stato', self.on_stato, 10)
        self.client = self.create_client(ParamSetV2, '/mavros/param/set')

        self.timer = self.create_timer(0.5, self.aggiorna)

        if self.modo == 'spoofing':
            self.get_logger().info(
                'Modo spoofing: falsificazione di {:.6f} gradi '
                '(~{:.0f} m). Atteso: il fix grezzo si sposta, la stima '
                'dell EKF no.'.format(self.spoofing_gradi,
                                      self.spoofing_gradi * 111320))
        elif self.modo not in MODI:
            self.get_logger().error(
                'modo "{}" sconosciuto: usare {} oppure spoofing'.format(
                    self.modo, ', '.join(MODI)))
        self.get_logger().info(
            'GnssDenialNode avviato — modo {}, {}'.format(
                self.modo,
                'attacco continuo' if self.sempre_attivo else
                '{:.0f}s attivo / {:.0f}s inattivo'.format(
                    self.durata_attivo_s, self.durata_inattivo_s)))

    # ------------------------------------------------------------- parametri

    def _scrivi(self, nome, valore, intero=True):
        """Scrive un parametro dell'autopilota senza bloccare il timer.

        La chiamata e asincrona: un servizio che non risponde bloccherebbe il
        nodo, e con esso la pubblicazione di /gps/denial_active, falsando
        l'annotazione delle finestre di attacco nei dati.
        """
        if not self.client.service_is_ready():
            self.get_logger().warn(
                '/mavros/param/set non pronto: attacco non applicato',
                throttle_duration_sec=10.0)
            return
        richiesta = ParamSetV2.Request()
        richiesta.param_id = nome
        richiesta.force_set = True
        if intero:
            richiesta.value = ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=int(valore))
        else:
            richiesta.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(valore))

        inizio = time.time()
        futuro = self.client.call_async(richiesta)
        self.chiamate_in_corso.append(futuro)

        def esito(f):
            latenza = (time.time() - inizio) * 1000.0
            risultato = f.result()
            if risultato is None or not risultato.success:
                self.get_logger().error(
                    '{} = {} rifiutato ({:.0f} ms)'.format(nome, valore, latenza))
            else:
                self.get_logger().info(
                    '{} = {} ({:.0f} ms)'.format(nome, valore, latenza))
            if f in self.chiamate_in_corso:
                self.chiamate_in_corso.remove(f)

        futuro.add_done_callback(esito)

    def _applica(self, attacco):
        """Attiva o disattiva l'attacco secondo il modo scelto."""
        if self.modo == 'spoofing':
            offset = self.spoofing_gradi if attacco else 0.0
            self._scrivi('SIM_GPS1_GLTCH_X', offset, intero=False)
            self._scrivi('SIM_GPS1_GLTCH_Y', offset, intero=False)
            return
        if self.modo not in MODI:
            return
        nome, valore_attacco, valore_riposo = MODI[self.modo]
        self._scrivi(nome, valore_attacco if attacco else valore_riposo)

    # -------------------------------------------------------------- callback

    def on_stato(self, msg: String):
        self.fase_missione = msg.data

    def _missione_partita(self):
        """Vero quando la missione e oltre l'attesa a terra.

        Prima non si tocca il GPS: l'arming e il decollo hanno bisogno di un
        fix valido, e negarlo li farebbe fallire per un motivo che non ha nulla
        a che vedere con l'esperimento.
        """
        return FaseMissione.ATTESA.value not in self.fase_missione

    def aggiorna(self):
        partita = self._missione_partita()

        if not partita:
            if self.attivo:
                self._applica(False)
                self.attivo = False
                self.istante_cambio = None
            self.pub.publish(Bool(data=False))
            return

        if self.failsafe_salvato is None and self.disattiva_failsafe_ekf:
            self.failsafe_salvato = 1     # valore di default di ArduPilot
            self.get_logger().warn(
                'Disattivo FS_EKF_ACTION: senza questo ArduPilot passerebbe in '
                'LAND alla perdita del GPS, scavalcando la missione. Legittimo '
                'solo in simulazione.')
            self._scrivi('FS_EKF_ACTION', 0)

        adesso = self.get_clock().now().nanoseconds / 1e9
        if self.istante_cambio is None:
            self.istante_cambio = adesso
            self.attivo = True
            self._applica(True)
        elif not self.sempre_attivo:
            durata = (self.durata_attivo_s if self.attivo
                      else self.durata_inattivo_s)
            if adesso - self.istante_cambio >= durata:
                self.attivo = not self.attivo
                self.istante_cambio = adesso
                self._applica(self.attivo)
                if self.attivo:
                    self.get_logger().warn('GNSS SOTTO ATTACCO ({})'.format(self.modo))
                else:
                    self.get_logger().info('GNSS ripristinato')

        self.pub.publish(Bool(data=self.attivo))

    # -------------------------------------------------------------- chiusura

    def ripristina(self):
        """Rimette il simulatore come lo si e trovato.

        Senza questo, un parametro di attacco resterebbe impostato e la prova
        successiva partirebbe con il GPS compromesso senza che nulla lo dica.
        """
        self.get_logger().info('Ripristino dei parametri del simulatore')
        self._applica(False)
        if self.failsafe_salvato is not None:
            self._scrivi('FS_EKF_ACTION', self.failsafe_salvato)
        # Le chiamate sono asincrone: si concede loro il tempo di partire.
        scadenza = time.time() + 3.0
        while self.chiamate_in_corso and time.time() < scadenza:
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = GnssDenialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ripristina()
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
