#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String, Float32
from drone_tracking.mission_node import FaseMissione  # type: ignore
from drone_tracking.parametri import parametro  # type: ignore
import numpy as np

class TrackerNode(Node):
    def __init__(self):
        super().__init__('tracker_node')

        self.noise_sub = self.create_subscription(
            Float32, '/rf/noise_level',
            self.on_noise_level, 10)

        self.sub = self.create_subscription(
            Point, '/target/jammed_position', self.on_detection, 10)

        self.pub = self.create_publisher(
            Point, '/target/tracked_position', 10)

        self.reset_sub = self.create_subscription(
            Bool, '/tracker/reset', self.on_reset, 10)

        self.mission_sub = self.create_subscription(
            String, '/mission/stato', self.on_stato_missione, 10)

        # --- Filtro Kalman [x, y, vx, vy] ---
        self.stato_stimato = np.zeros((4, 1), dtype=np.float32)

        # Il nodo è guidato dai messaggi, non da un timer: il suo ritmo è quello
        # della telecamera, che varia col carico della macchina (misurato fra 5 e
        # 13 Hz). Il dt viene quindi ricavato dai tempi reali fra due misure, non
        # fissato a una costante.
        self.dt_nominale = 0.1      # usato solo per la primissima misura
        self.dt_min      = 0.02     # limiti di sicurezza: un dt anomalo
        self.dt_max      = 0.5      # manderebbe in divergenza la predizione
        self.ultimo_istante = None

        self.evoluzione_stato = np.array([
            [1, 0, self.dt_nominale, 0               ],
            [0, 1, 0,                self.dt_nominale],
            [0, 0, 1,                0               ],
            [0, 0, 0,                1               ]
        ], dtype=np.float32)

        self.mappa_osservazione = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float32)

        self.livello_rumore = 0.0
        
        # Intensita del rumore di accelerazione del modello, in (unita
        # normalizzate)^2/s^3. Non e piu una matrice costante: la matrice Q
        # viene ricostruita a ogni predizione in funzione del dt effettivo, con
        # la discretizzazione standard di un modello a velocita quasi costante
        # (vedi _matrice_Q). Prima veniva sommata sempre la stessa Q qualunque
        # fosse il tempo trascorso: al ritmo variabile della telecamera
        # (misurato fra 5 e 13 Hz) lo stesso intervallo veniva penalizzato o
        # premiato a caso, e i termini incrociati posizione-velocita, che in
        # questo modello esistono, mancavano del tutto.
        #
        # Il valore e scelto per riprodurre al dt nominale di 0.1 s il termine
        # di velocita della vecchia matrice (0.5 = q*dt con q = 5.0), cosi il
        # confronto con le prove precedenti resta leggibile. Il termine di
        # posizione risulta invece piu piccolo di quanto fosse (q*dt^3/3 =
        # 0.0017 contro 0.01), che e corretto: la posizione non e affetta da
        # rumore proprio, eredita solo quello dell'accelerazione integrata due
        # volte.
        self.intensita_rumore_accel = parametro(
            self, 'intensita_rumore_accel', 5.0)
        # Incertezza della misura, che cresce con il rumore dichiarato sul
        # datalink: e il meccanismo con cui il filtro si fida meno del
        # rilevamento durante il jamming (vedi on_noise_level).
        self.rumore_sensore_base = parametro(self, 'rumore_sensore_base', 0.05)
        self.rumore_sensore_max  = parametro(self, 'rumore_sensore_max', 2.0)
        self.incertezza_sensore = (np.eye(2, dtype=np.float32)
                                   * self.rumore_sensore_base)
        self.incertezza_corrente = np.eye(4, dtype=np.float32)

        self.bersaglio_acquisito = False
        self.frame_senza_segnale = 0
        # Quanti frame senza segnale tollerare continuando a pubblicare la
        # predizione. Alzata da 5 a 15 (~1.4 s a 11 Hz): il controller insegue
        # la stima del filtro, quindi finche questa resta valida il drone
        # continua a rincorrere il bersaglio invece di fermarsi. Serve nei casi
        # difficili, come una fuga nella direzione opposta a quella in cui il
        # drone si sta muovendo, dove il bersaglio esce dall'inquadratura per
        # qualche decimo di secondo mentre il velivolo inverte la marcia.
        self.soglia_perdita = parametro(self, 'soglia_perdita', 15)

        # Ultima area valida del contorno. Serve a marcare come utilizzabili le
        # posizioni predette durante una perdita di segnale: `z` è il flag di
        # validità letto a valle, e ricopiare lo zero del messaggio in ingresso
        # le farebbe scartare come "bersaglio assente".
        self.ultima_area = 0.0

        self.get_logger().info('TrackerNode avviato — filtro Kalman attivo')

    def _matrice_Q(self, dt):
        """Rumore di processo per un modello a velocita quasi costante.

        Un'accelerazione ignota di intensita q, integrata su un intervallo dt,
        produce sulla posizione una varianza q*dt^3/3, sulla velocita q*dt e fra
        le due una covarianza q*dt^2/2. Lo stato e [x, y, vx, vy], quindi i due
        assi occupano righe alternate e i termini incrociati stanno fuori dalla
        diagonale.
        """
        q = self.intensita_rumore_accel
        p = q * dt ** 3 / 3.0    # posizione
        c = q * dt ** 2 / 2.0    # posizione-velocita
        v = q * dt               # velocita
        return np.array([
            [p, 0, c, 0],
            [0, p, 0, c],
            [c, 0, v, 0],
            [0, c, 0, v],
        ], dtype=np.float32)

    def _calcola_dt(self):
        """Intervallo reale trascorso dall'ultima misura, con clamp di sicurezza."""
        adesso = self.get_clock().now().nanoseconds / 1e9
        if self.ultimo_istante is None:
            self.ultimo_istante = adesso
            return self.dt_nominale
        dt = adesso - self.ultimo_istante
        self.ultimo_istante = adesso
        return float(min(max(dt, self.dt_min), self.dt_max))

    def on_noise_level(self, msg: Float32):
        self.livello_rumore = msg.data
        r_dinamico = (self.rumore_sensore_base
                      + (self.rumore_sensore_max - self.rumore_sensore_base)
                      * self.livello_rumore)
        self.incertezza_sensore = np.eye(2, dtype=np.float32) * r_dinamico
        # self.get_logger().info(f'R adattivo: {r_dinamico:.3f} (rumore RF: {self.livello_rumore:.1f})')
        
    def on_stato_missione(self, msg: String):
        if FaseMissione.ATTESA.value in msg.data:
            self._reset()

    def on_reset(self, msg: Bool):
        if msg.data:
            self._reset()
            self.get_logger().info('Tracker resettato')

    def _reset(self):
        self.bersaglio_acquisito = False
        self.frame_senza_segnale = 0
        self.stato_stimato = np.zeros((4, 1), dtype=np.float32)
        self.incertezza_corrente = np.eye(4, dtype=np.float32)
        self.ultima_area = 0.0
        # Alla ripresa il primo dt ripartirebbe dal tempo trascorso durante la
        # perdita, che non è un intervallo di campionamento valido.
        self.ultimo_istante = None

    def on_detection(self, msg: Point):
        segnale_presente = not (msg.z == 0.0)

        if not self.bersaglio_acquisito and segnale_presente:
            self.stato_stimato = np.array(
                [[msg.x], [msg.y], [0.0], [0.0]], dtype=np.float32)
            self.bersaglio_acquisito = True
            self.ultima_area = msg.z
            self._calcola_dt()   # inizializza il riferimento temporale
            self.get_logger().info('Bersaglio acquisito')
            # La posizione appena acquisita va pubblicata subito: uscire senza
            # farlo faceva perdere un messaggio a ogni riacquisizione, e sotto
            # jamming le riacquisizioni sono continue.
            self.pub.publish(Point(x=float(msg.x), y=float(msg.y), z=float(msg.z)))
            return

        if not self.bersaglio_acquisito:
            # Continua a pubblicare coordinate nulle se non ha ancora agganciato nulla
            msg_vuoto = Point(x=0.0, y=0.0, z=0.0)
            self.pub.publish(msg_vuoto)
            return

        # PREDIZIONE KALMAN — dt aggiornato al ritmo effettivo della catena
        dt = self._calcola_dt()
        self.evoluzione_stato[0, 2] = dt
        self.evoluzione_stato[1, 3] = dt

        self.stato_stimato = self.evoluzione_stato @ self.stato_stimato
        self.incertezza_corrente = (
            self.evoluzione_stato @ self.incertezza_corrente
            @ self.evoluzione_stato.T + self._matrice_Q(dt)
        )

        if segnale_presente:
            self.frame_senza_segnale = 0
            misura = np.array([[msg.x], [msg.y]], dtype=np.float32)

            S = (self.mappa_osservazione @ self.incertezza_corrente
                 @ self.mappa_osservazione.T + self.incertezza_sensore)

            guadagno_kalman = (self.incertezza_corrente
                               @ self.mappa_osservazione.T
                               @ np.linalg.inv(S))

            errore = misura - self.mappa_osservazione @ self.stato_stimato
            self.stato_stimato = self.stato_stimato + guadagno_kalman @ errore
            self.incertezza_corrente = (
                (np.eye(4) - guadagno_kalman @ self.mappa_osservazione)
                @ self.incertezza_corrente
            )

            # Qui la velocità stimata veniva moltiplicata per 0.6 a ogni
            # aggiornamento, per "ridurre predizioni errate". Rimosso: era uno
            # smorzamento applicato allo stato senza toccare la covarianza
            # corrispondente, cioè il filtro dichiarava una fiducia che non
            # corrispondeva più alla stima, e la coerenza fra le due è l'unica
            # cosa che rende ottimo un filtro di Kalman. Peggio, essendo
            # applicato a ogni misura, il fattore si componeva: dopo dieci
            # aggiornamenti la velocità era ridotta a 0.6^10, cioè lo 0.6% del
            # valore stimato, e la predizione durante una perdita di segnale
            # restava praticamente ferma sull'ultima posizione invece di
            # estrapolare il moto del bersaglio.
            # Lo stesso effetto — stima di velocità meno nervosa — si ottiene
            # ora per la via corretta, cioè dall'intensità di rumore del modello
            # in _matrice_Q, che governa quanto la velocità può cambiare fra due
            # misure e aggiorna di conseguenza anche l'incertezza.

            self.ultima_area = msg.z

            # Pubblica la posizione stimata aggiornata
            posizione_stimata = Point()
            posizione_stimata.x = float(self.stato_stimato[0].item())
            posizione_stimata.y = float(self.stato_stimato[1].item())
            posizione_stimata.z = float(msg.z)
            self.pub.publish(posizione_stimata)
        else:
            self.frame_senza_segnale += 1
            if self.frame_senza_segnale > self.soglia_perdita:
                self._reset()
                self.get_logger().warn('Bersaglio perso — reset tracker')
                
                # Invia il segnale di stop/perdita a mission_node e controller_node
                msg_perso = Point(x=0.0, y=0.0, z=0.0)
                self.pub.publish(msg_perso)
                return
            
            # Pubblica la predizione per tollerare micro-interruzioni.
            # `z` porta l'ultima area valida, non lo zero del messaggio in
            # ingresso: la stima è utilizzabile e va marcata come tale.
            posizione_stimata = Point()
            posizione_stimata.x = float(self.stato_stimato[0].item())
            posizione_stimata.y = float(self.stato_stimato[1].item())
            posizione_stimata.z = float(self.ultima_area)
            self.pub.publish(posizione_stimata)
            # self.get_logger().info(
            #     f'Tentativo predizione — Frame persi: {self.frame_senza_segnale}/{self.soglia_perdita}')

def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()