#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from enum import Enum
import math

from drone_tracking.parametri import parametro  # type: ignore

class FaseMissione(Enum):
    ATTESA = "ATTESA"
    PATTUGLIAMENTO = "PATTUGLIAMENTO"
    AGGANCIO = "AGGANCIO"
    RICERCA = "RICERCA"


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        # Per la gestione pacchetti persi
        qos_mavros = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriber: posizione drone
        self.pos_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.on_position, qos_mavros)
        
        # Subscriber: Avvio di missione
        self.start_sub = self.create_subscription(
            Bool, '/mission/avvia',
            self.on_avvia, 10)
        
        # Subscriber: bersaglio rilevato
        self.target_sub = self.create_subscription(
            Point, '/target/tracked_position',
            self.on_target, 10)

        # Publisher: waypoint verso MAVROS2
        self.waypoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)

        # Publisher: stato missione
        self.stato_pub = self.create_publisher(
            String, '/mission/stato', 10)

        # Publisher: per resettare lo stato missione
        self.reset_pub = self.create_publisher(Bool, '/tracker/reset', 10)
        
        # Il drone va dritto verso la zona di oscillazione (8, 0)
        self.waypoints = [
            ( 0.0,  0.0, 12.0),   # origine - partenza
            (20.0,  0.0, 12.0),   # nord
            (20.0, 20.0, 12.0),   # nord-est - pallina qui
            ( 0.0, 20.0, 12.0),   # est
            ( 0.0,  0.0, 12.0),   # ritorno
        ]

        self.waypoint_corrente = 0
        self.posizione_attuale = None
        self.bersaglio_agganciato = False

        # Istante dell'ultimo messaggio ricevuto da ciascuna sorgente. Servono
        # per distinguere "il bersaglio non e visibile" da "nessuno mi sta piu
        # dicendo se il bersaglio e visibile": il primo caso era gestito, il
        # secondo no, e la differenza pratica e che nel secondo la missione
        # restava congelata in AGGANCIO a tempo indeterminato.
        self.istante_ultimo_target = None
        self.istante_ultima_posa   = None
        self.timeout_percezione_s = parametro(
            self, 'timeout_percezione_s', 0.5)   # ~5 messaggi al ritmo camera
        self.timeout_telemetria_s = parametro(
            self, 'timeout_telemetria_s', 1.0)   # MAVROS pubblica a 10-50 Hz
        
        # Tolleranza sul waypoint. Alzata da 1.2 a 3.0 m: con 1.2 il drone doveva
        # arrivare quasi fermo per centrare il punto, e a velocita di crociera lo
        # sorpassava e tornava indietro, oscillando. Tre metri su un circuito di
        # 20 non cambiano il percorso e permettono di non frenare.
        self.soglia_waypoint = parametro(self, 'soglia_waypoint', 3.0)
        
        self.rilevamento_attivo = False
        self.fase = FaseMissione.ATTESA

        self.frame_conferma_richiesti = parametro(
            self, 'frame_conferma_richiesti', 5)
        # In RICERCA il bersaglio attraversa il campo visivo di sfuggita: cinque
        # frame consecutivi (~0.45 s) sono spesso piu di quanto duri il
        # passaggio, e il riaggancio non scattava mai. Per riagganciare bastano
        # meno conferme: il rischio di un falso positivo e accettabile, visto che
        # l'alternativa e continuare a cercare a vuoto.
        self.frame_conferma_riaggancio = parametro(
            self, 'frame_conferma_riaggancio', 2)
        self.frame_bersaglio_visibile = 0
        
        self.timer = self.create_timer(0.5, self.aggiorna_missione)
        self.get_logger().info('MissionNode avviato — in attesa di arming')
        
        # Variabili per la ricerca bersaglio se non agganciato.
        # Velocità in unità al secondo, integrate sul dt reale: l'espansione era
        # un incremento per chiamata (0.002) su un timer a 2 Hz, cioè 4 mm/s.
        # La spirale impiegava oltre un'ora ad allargarsi di 20 m e in pratica
        # restava un cerchio fisso di raggio 3 m, mentre il bersaglio in fuga si
        # allontanava a 1.2 m/s: non lo raggiungeva mai.
        self.ricerca_centro_x = 0.0
        self.ricerca_centro_y = 0.0
        self.ricerca_raggio   = 3.0
        self.ricerca_t        = 0.0
        self.ricerca_espansione = 0.0
        self.ricerca_vel_angolare = parametro(
            self, 'ricerca_vel_angolare', 0.35)   # rad/s
        # 0.4 m/s di espansione su un giro da ~18 s fanno ~7 m fra un braccio e
        # il successivo: meno dei ~15 m inquadrati a 12 m di quota, quindi la
        # spirale non lascia zone scoperte.
        self.ricerca_vel_espansione = parametro(
            self, 'ricerca_vel_espansione', 0.4)   # m/s
        self.ricerca_raggio_max = parametro(
            self, 'ricerca_raggio_max', 25.0)      # m, poi si rinuncia
        self.ricerca_ultimo_istante = None
        # Attesa prima di dichiarare perso il bersaglio, in SECONDI. Era un
        # conteggio di frame tarato su 10 Hz, ma /target/tracked_position segue
        # il ritmo della telecamera (5-13 Hz): la stessa soglia valeva fra 1.5 e
        # 4 secondi a seconda del carico della macchina.
        self.istante_perdita     = None
        # Portata a 6 s nell'idea che il controller continuasse a inseguire per
        # tutta l'attesa. Misurando la sequenza reale si e visto che non e cosi:
        # il tracker estrapola per 15 fotogrammi (~1.4 s), poi si azzera, e da
        # quel momento controller_node azzerava il comando. Il drone passava
        # quindi ~1.4 s a inseguire e i restanti ~4.6 s immobile, prima di
        # cominciare a cercare davvero.
        # Ora l'attesa e coperta in modo attivo: controller_node prosegue con
        # una rampa di coasting (durata_coasting_s, 2 s) nella direzione in cui
        # il bersaglio si stava muovendo. Restare fermi oltre non aggiunge
        # probabilita di riacquisizione, mentre la spirale di RICERCA almeno si
        # muove: 3 s coprono la predizione piu il coasting e lasciano circa
        # mezzo secondo di margine.
        self.soglia_avvia_ricerca_s = parametro(
            self, 'soglia_avvia_ricerca_s', 3.0)

    def on_position(self, msg: PoseStamped):
        self.istante_ultima_posa = self.get_clock().now().nanoseconds / 1e9
        self.posizione_attuale = msg.pose.position
        
        # self.get_logger().info(f'Pos: x:{self.posizione_attuale.x:.1f} y:{self.posizione_attuale.y:.1f}')

    def _fresco(self, istante, limite):
        """Vero se quella sorgente ha parlato di recente."""
        if istante is None:
            return False
        return (self.get_clock().now().nanoseconds / 1e9) - istante < limite

    def _valuta_perdita_aggancio(self, target_visibile):
        """Contabilizza il tempo senza bersaglio in AGGANCIO e, oltre la
        soglia, passa a RICERCA.

        Chiamata sia all'arrivo dei messaggi del tracker sia dal timer
        periodico. Il secondo caso e quello che prima mancava: se
        /target/tracked_position tace del tutto — detector fermo, ponte delle
        immagini caduto — non arrivava nessun messaggio a far partire il
        conteggio, e la fase restava AGGANCIO per sempre con il drone in attesa
        di un bersaglio che nessuno stava piu cercando.
        """
        if target_visibile:
            self.istante_perdita = None
            return

        adesso = self.get_clock().now().nanoseconds / 1e9
        if self.istante_perdita is None:
            self.istante_perdita = adesso
        elif adesso - self.istante_perdita > self.soglia_avvia_ricerca_s:
            self.get_logger().warn('Bersaglio perso — avvio ricerca')
            if self.posizione_attuale:
                self.ricerca_centro_x = self.posizione_attuale.x
                self.ricerca_centro_y = self.posizione_attuale.y
            self.ricerca_t = 0.0
            self.ricerca_espansione = 0.0
            self.ricerca_ultimo_istante = None
            self.fase = FaseMissione.RICERCA
            self.bersaglio_agganciato = False
            self.istante_perdita = None
            self.frame_bersaglio_visibile = 0

    def on_target(self, msg: Point):
        self.istante_ultimo_target = self.get_clock().now().nanoseconds / 1e9
        target_visibile = (msg.x != 0.0 or msg.y != 0.0)
        altitudine_ok = (self.posizione_attuale is not None
                        and self.posizione_attuale.z > 2.0)
        in_pattugliamento = self.fase == FaseMissione.PATTUGLIAMENTO

        # Gestione perdita bersaglio in fase AGGANCIO
        if self.fase == FaseMissione.AGGANCIO:
            self._valuta_perdita_aggancio(target_visibile)
            return

        # Gestione riaggancio in fase RICERCA
        if self.fase == FaseMissione.RICERCA:
            if target_visibile:
                self.frame_bersaglio_visibile += 1
                if self.frame_bersaglio_visibile >= self.frame_conferma_riaggancio:
                    self.get_logger().warn('Bersaglio riagganciato')
                    self.bersaglio_agganciato = True
                    self.istante_perdita = None
                    self.fase = FaseMissione.AGGANCIO
            else:
                self.frame_bersaglio_visibile = 0
            return

        # Logica pattugliamento esistente
        if not (in_pattugliamento and altitudine_ok and self.rilevamento_attivo):
            self.frame_bersaglio_visibile = 0
            return

        if target_visibile:
            self.frame_bersaglio_visibile += 1
        else:
            self.frame_bersaglio_visibile = 0

        if (self.frame_bersaglio_visibile >= self.frame_conferma_richiesti
                and not self.bersaglio_agganciato):
            self.bersaglio_agganciato = True
            self.fase = FaseMissione.AGGANCIO
            self.get_logger().warn('Bersaglio agganciato')

    def distanza_waypoint(self, target):
        if self.posizione_attuale is None:
            return float('inf')
        dx = self.posizione_attuale.x - target[0]
        dy = self.posizione_attuale.y - target[1]
        dz = self.posizione_attuale.z - target[2]
        return (dx**2 + dy**2 + dz**2) ** 0.5

    def pubblica_waypoint(self, wp):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = wp[0]
        msg.pose.position.y = wp[1]
        msg.pose.position.z = wp[2]
        msg.pose.orientation.w = 1.0
        self.waypoint_pub.publish(msg)

    def aggiorna_missione(self):
        stato_msg = String()

        if self.fase == FaseMissione.ATTESA:
            stato_msg.data = FaseMissione.ATTESA.value
            self.stato_pub.publish(stato_msg)
            return

        # La missione e avviata: da qui in poi serve sapere dove si trova il
        # drone. Senza la posa, distanza_waypoint restituisce infinito e il
        # pattugliamento non avanza di un solo waypoint; prima accadeva in
        # silenzio, con il drone fermo e nessuna indicazione del perche.
        if not self._fresco(self.istante_ultima_posa, self.timeout_telemetria_s):
            self.get_logger().error(
                'Nessuna posa da /mavros/local_position/pose da oltre '
                '{:.1f}s: la missione non puo avanzare. MAVROS e attivo e '
                'ArduPilot invia gli stream di posizione?'.format(
                    self.timeout_telemetria_s),
                throttle_duration_sec=5.0)

        if self.fase == FaseMissione.AGGANCIO:
            stato_msg.data = FaseMissione.AGGANCIO.value
            self.stato_pub.publish(stato_msg)
            # Assenza di messaggi dal tracker: trattata come bersaglio non
            # visibile, non come stato congelato.
            if not self._fresco(self.istante_ultimo_target,
                                self.timeout_percezione_s):
                self.get_logger().error(
                    'Nessun messaggio da /target/tracked_position da oltre '
                    '{:.1f}s — lo tratto come bersaglio non visibile'.format(
                        self.timeout_percezione_s),
                    throttle_duration_sec=2.0)
                self._valuta_perdita_aggancio(False)
            return

        if self.fase == FaseMissione.RICERCA:
            stato_msg.data = FaseMissione.RICERCA.value
            self.stato_pub.publish(stato_msg)
            self.esegui_ricerca()
            return
        
        if self.waypoint_corrente >= len(self.waypoints):
            self.waypoint_corrente = 1  # Ricomincia il pattugliamento se non aggancia

        wp = self.waypoints[self.waypoint_corrente]
        self.pubblica_waypoint(wp)

        dist = self.distanza_waypoint(wp)
        self.get_logger().info(
            f'Waypoint {self.waypoint_corrente}/{len(self.waypoints)-1} '
            f'→ ({wp[0]:.0f},{wp[1]:.0f},{wp[2]:.0f})m '
            f'dist:{dist:.1f}m')

        if dist < self.soglia_waypoint:
            self.get_logger().info(f'Waypoint {self.waypoint_corrente} raggiunto vicino alla zona di oscillazione!')
            self.waypoint_corrente += 1

        stato_msg.data = f'{FaseMissione.PATTUGLIAMENTO.value}:{self.waypoint_corrente}'
        self.stato_pub.publish(stato_msg)

    def avvia_pattugliamento(self):
        self.fase = FaseMissione.PATTUGLIAMENTO
        self.waypoint_corrente = 0
        self.bersaglio_agganciato = False
        self.rilevamento_attivo = False
        
        reset_msg = Bool()
        reset_msg.data = True
        self.reset_pub.publish(reset_msg)
        
        # Timer per dare tempo al drone di decollare prima di attivare la visione
        self.crea_timer_visione = self.create_timer(2.0, self.abilita_rilevamento)
        self.get_logger().info('Pattugliamento avviato!')

    def abilita_rilevamento(self):
        self.rilevamento_attivo = True
        self.get_logger().info('Rilevamento bersaglio attivo')
        if hasattr(self, 'crea_timer_visione'):
            self.crea_timer_visione.destroy() # Distrugge il timer usa-e-getta per non accumulare callback
    
    def on_avvia(self, msg: Bool):
        self.get_logger().info(f'Ricevuto avvia: {msg.data} fase: {self.fase}')
        if msg.data and self.fase == FaseMissione.ATTESA:
            self.avvia_pattugliamento()

    def esegui_ricerca(self):
        # Spirale espandibile intorno all'ultima posizione nota
        adesso = self.get_clock().now().nanoseconds / 1e9
        if self.ricerca_ultimo_istante is None:
            dt = 0.5
        else:
            dt = min(max(adesso - self.ricerca_ultimo_istante, 0.05), 2.0)
        self.ricerca_ultimo_istante = adesso

        self.ricerca_t += self.ricerca_vel_angolare * dt
        self.ricerca_espansione += self.ricerca_vel_espansione * dt

        raggio_corrente = self.ricerca_raggio + self.ricerca_espansione

        # Oltre il raggio massimo la ricerca è considerata fallita e si torna a
        # pattugliare: continuare ad allargarsi porterebbe il drone sempre più
        # lontano dall'area di interesse, senza speranza di ritrovare nulla.
        if raggio_corrente > self.ricerca_raggio_max:
            self.get_logger().warn(
                f'Ricerca fallita entro {self.ricerca_raggio_max:.0f} m — '
                f'riprendo il pattugliamento')
            self.fase = FaseMissione.PATTUGLIAMENTO
            self.waypoint_corrente = 1
            self.ricerca_espansione = 0.0
            self.ricerca_t = 0.0
            self.ricerca_ultimo_istante = None
            self.frame_bersaglio_visibile = 0
            return

        x = self.ricerca_centro_x + raggio_corrente * math.cos(self.ricerca_t)
        y = self.ricerca_centro_y + raggio_corrente * math.sin(self.ricerca_t)
        z = self.waypoints[1][2]  # mantieni altitudine di crociera

        self.pubblica_waypoint((x, y, z))
        self.get_logger().info(
            f'RICERCA spirale -> ({x:.1f}, {y:.1f}) raggio:{raggio_corrente:.1f}m')

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()