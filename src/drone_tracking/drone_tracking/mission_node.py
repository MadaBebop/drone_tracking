#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
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

        # Subscriber: rilevamenti veri, quelli che il tracker riceve in
        # ingresso. Non `/target/position`: fra i due c'e il jammer, e contare
        # i rilevamenti puliti farebbe confermare un aggancio su
        # un'informazione che il filtro non ha mai avuto.
        self.create_subscription(
            Point, '/target/jammed_position', self.on_rilevamento, 10)

        # Subscriber: stima del bersaglio in metri, pubblicata dal controllo.
        # E cio che rende possibile cercare nella direzione giusta invece che
        # a spirale isotropa.
        self.create_subscription(
            Odometry, '/target/odometria', self.on_stima_bersaglio, 10)

        # Publisher: waypoint verso MAVROS2
        self.waypoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)

        # Publisher: stato missione
        self.stato_pub = self.create_publisher(
            String, '/mission/stato', 10)

        # Publisher: per resettare lo stato missione
        self.reset_pub = self.create_publisher(Bool, '/tracker/reset', 10)
        
        # Circuito da 150 m a 50 m di quota: l'impronta a terra vale 100 m, che
        # un veicolo a 15 m/s attraversa in quasi sette secondi.
        self.waypoints = [
            (  0.0,   0.0, 50.0),   # origine - partenza
            (150.0,   0.0, 50.0),   # nord
            (150.0, 150.0, 50.0),   # nord-est - il bersaglio pattuglia qui
            (  0.0, 150.0, 50.0),   # est
            (  0.0,   0.0, 50.0),   # ritorno
        ]

        self.waypoint_corrente = 0
        self.posizione_attuale = None
        self.bersaglio_agganciato = False

        # Distinguono «il bersaglio non e visibile» da «nessuno mi sta piu
        # dicendo se e visibile»: senza, il silenzio di una sorgente congelava
        # la missione in AGGANCIO a tempo indeterminato.
        self.istante_ultimo_target = None
        self.istante_ultima_posa   = None
        # Ultima stima nota del bersaglio nel mondo: (x, y, vx, vy, istante).
        # Sopravvive alla perdita dell'aggancio, ed e proprio allora che serve.
        self.stima_bersaglio = None
        self.timeout_percezione_s = parametro(
            self, 'timeout_percezione_s', 0.5)   # ~5 messaggi al ritmo camera
        self.timeout_telemetria_s = parametro(
            self, 'timeout_telemetria_s', 1.0)   # MAVROS pubblica a 10-50 Hz
        
        # Tolleranza sul waypoint: deve valere qualche decimo di secondo di
        # volo, altrimenti il drone dovrebbe arrivarci quasi fermo. Quindici
        # metri su una tratta di 150 non cambiano il percorso.
        self.soglia_waypoint = parametro(self, 'soglia_waypoint', 15.0)
        
        self.rilevamento_attivo = False
        self.fase = FaseMissione.ATTESA

        self.frame_conferma_richiesti = parametro(
            self, 'frame_conferma_richiesti', 5)
        # In RICERCA il bersaglio attraversa il campo di sfuggita, spesso per
        # meno di quanto durino cinque fotogrammi: per riagganciare bastano meno
        # conferme, e il rischio di un falso positivo e preferibile al cercare
        # a vuoto.
        self.frame_conferma_riaggancio = parametro(
            self, 'frame_conferma_riaggancio', 2)
        # Contati sul flusso del RILEVATORE e non del filtro: quello resta
        # valido per `soglia_perdita` fotogrammi dopo l'ultima misura, quindi un
        # solo avvistamento soddisferebbe qualunque soglia di conferma.
        self.rilevamenti_consecutivi = 0
        
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
        # Raggio iniziale della spirale: dell'ordine dell'impronta a terra, che
        # a 50 m di quota vale 100 m. Partire da 3 m come nello scenario
        # ridotto significherebbe cercare dentro un'area gia inquadrata.
        self.ricerca_raggio   = 30.0
        self.ricerca_t        = 0.0
        self.ricerca_espansione = 0.0
        # 0.25 rad/s su raggio 30 m fanno 7.5 m/s tangenziali, entro le
        # possibilita del velivolo.
        self.ricerca_vel_angolare = parametro(
            self, 'ricerca_vel_angolare', 0.25)   # rad/s
        # Un giro dura 2*pi/0.25 = 25 s: a 3 m/s di espansione i bracci distano
        # 75 m, meno dei 100 inquadrati, quindi non restano zone scoperte.
        self.ricerca_vel_espansione = parametro(
            self, 'ricerca_vel_espansione', 3.0)   # m/s
        # Un bersaglio a 15 m/s percorre 300 m nei venti secondi di fuga: il
        # raggio massimo deve essere dello stesso ordine, altrimenti la ricerca
        # rinuncia prima di aver coperto la zona in cui il bersaglio puo
        # ragionevolmente trovarsi.
        self.ricerca_raggio_max = parametro(
            self, 'ricerca_raggio_max', 300.0)     # m, poi si rinuncia
        self.ricerca_ultimo_istante = None
        # Primo tempo della ricerca: volare dove il bersaglio sarebbe se avesse
        # proseguito, estrapolando dalla velocita stimata. Spento per default
        # perche quella stima aveva errore pari al segnale (README, «Ricerca del
        # bersaglio»), e da una grandezza cosi non si ricava una direzione. Il
        # difetto era a monte ed e stato corretto — il filtro ora riceve il moto
        # del velivolo come ingresso noto — quindi il valore va riprovato: 8.0 s
        # e il punto di partenza ragionevole, oltre i quali l'estrapolazione
        # rettilinea decade perche un veicolo in fuga curva.
        #
        # L'altra meta della modifica resta ACCESA: il centro della ricerca
        # sull'ultima posizione nota del bersaglio, che non dipende dalla
        # velocita.
        self.durata_inseguimento_cieco_s = parametro(
            self, 'durata_inseguimento_cieco_s', 0.0)
        self.istante_inizio_ricerca = None
        self.istante_perdita = None
        # Attesa prima di dichiarare perso il bersaglio, in SECONDI e non in
        # fotogrammi: il ritmo della telecamera varia col carico, e una soglia
        # contata in fotogrammi valeva fra 1.5 e 4 secondi. Tre secondi coprono
        # la predizione del filtro piu la rampa di coasting del controllo;
        # restare fermi oltre non aggiunge probabilita di riacquisizione,
        # mentre la spirale almeno si muove.
        self.soglia_avvia_ricerca_s = parametro(
            self, 'soglia_avvia_ricerca_s', 3.0)

    def on_rilevamento(self, msg: Point):
        """Rilevamenti consecutivi del bersaglio.

        La convenzione del rilevatore e l'area: z a zero significa che non ha
        trovato nulla. Si guarda quella e non x/y, perche un bersaglio
        esattamente al centro dell'inquadratura ha x = y = 0 pur essendo
        perfettamente visibile.
        """
        if msg.z != 0.0:
            self.rilevamenti_consecutivi += 1
        else:
            self.rilevamenti_consecutivi = 0

    def on_stima_bersaglio(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        # La bandiera dice se la velocita e ricostruita o solo zero: senza,
        # l'inseguimento cieco volerebbe verso l'ultima posizione nota invece
        # che verso dove il bersaglio sta andando, che e la stessa cosa che
        # faceva la spirale.
        velocita_valida = msg.twist.covariance[0] > 0.5
        self.stima_bersaglio = (
            p.x, p.y,
            v.x if velocita_valida else 0.0,
            v.y if velocita_valida else 0.0,
            self.get_clock().now().nanoseconds / 1e9)

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
            # Il centro della ricerca e l'ultima posizione nota del
            # BERSAGLIO, non quella del drone: a velocita reali le due
            # differiscono di decine di metri, e cercare attorno a se stessi
            # significa cercare dove il bersaglio non e.
            if self.stima_bersaglio is not None:
                self.ricerca_centro_x = self.stima_bersaglio[0]
                self.ricerca_centro_y = self.stima_bersaglio[1]
                self.get_logger().warn(
                    'Bersaglio perso — ricerca da ({:.0f}, {:.0f}) '
                    'con velocita ({:+.1f}, {:+.1f}) m/s'.format(
                        self.stima_bersaglio[0], self.stima_bersaglio[1],
                        self.stima_bersaglio[2], self.stima_bersaglio[3]))
            elif self.posizione_attuale:
                self.ricerca_centro_x = self.posizione_attuale.x
                self.ricerca_centro_y = self.posizione_attuale.y
                self.get_logger().warn(
                    'Bersaglio perso — nessuna stima disponibile, '
                    'ricerca attorno alla posizione del drone')
            self.istante_inizio_ricerca = adesso
            self.ricerca_t = 0.0
            self.ricerca_espansione = 0.0
            self.ricerca_ultimo_istante = None
            self.fase = FaseMissione.RICERCA
            self.bersaglio_agganciato = False
            self.istante_perdita = None
            self.rilevamenti_consecutivi = 0

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

        # Gestione riaggancio in fase RICERCA. La condizione e sui
        # RILEVAMENTI consecutivi: `target_visibile` qui sarebbe vero anche
        # per una predizione, e un lampo di un fotogramma ne genera a
        # sufficienza per qualunque soglia.
        if self.fase == FaseMissione.RICERCA:
            if self.rilevamenti_consecutivi >= self.frame_conferma_riaggancio:
                self.get_logger().warn(
                    'Bersaglio riagganciato ({} rilevamenti consecutivi)'.format(
                        self.rilevamenti_consecutivi))
                self.bersaglio_agganciato = True
                self.istante_perdita = None
                self.fase = FaseMissione.AGGANCIO
            return

        # Logica pattugliamento esistente
        if not (in_pattugliamento and altitudine_ok and self.rilevamento_attivo):
            self.rilevamenti_consecutivi = 0
            return

        # Stesso criterio dell'aggancio iniziale: entrare in inseguimento
        # richiede di aver visto il bersaglio, non di averlo predetto. Qui il
        # difetto non si manifestava — sopra il bersaglio i rilevamenti sono
        # continui — ma il criterio sbagliato era lo stesso, e due criteri
        # diversi per la stessa decisione sono un difetto in attesa.
        if (self.rilevamenti_consecutivi >= self.frame_conferma_richiesti
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
        adesso = self.get_clock().now().nanoseconds / 1e9
        quota = self.waypoints[1][2]

        # --- Primo tempo: inseguimento cieco ---
        # Si vola dove il bersaglio sarebbe se avesse proseguito dritto. E
        # l'unica informazione direzionale disponibile, e a velocita reali vale
        # piu di qualunque strategia di copertura: la spirale si espande a
        # pochi metri al secondo mentre il bersaglio ne percorre quindici.
        if (self.stima_bersaglio is not None
                and self.istante_inizio_ricerca is not None
                and adesso - self.istante_inizio_ricerca
                < self.durata_inseguimento_cieco_s):
            x0, y0, vx, vy, t0 = self.stima_bersaglio
            trascorso = adesso - t0
            x = x0 + vx * trascorso
            y = y0 + vy * trascorso
            # La spirale, se servira, partira da qui e non dal punto di
            # perdita.
            self.ricerca_centro_x, self.ricerca_centro_y = x, y
            self.pubblica_waypoint((x, y, quota))
            self.get_logger().info(
                'RICERCA inseguimento cieco -> ({:.0f}, {:.0f}) '
                'da {:.1f}s'.format(x, y, adesso - self.istante_inizio_ricerca))
            return

        # --- Secondo tempo: spirale attorno alla posizione estrapolata ---
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
            self.rilevamenti_consecutivi = 0
            return

        x = self.ricerca_centro_x + raggio_corrente * math.cos(self.ricerca_t)
        y = self.ricerca_centro_y + raggio_corrente * math.sin(self.ricerca_t)

        self.pubblica_waypoint((x, y, quota))
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