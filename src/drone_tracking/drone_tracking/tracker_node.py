#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String, Float32
from drone_tracking.mission_node import FaseMissione  # type: ignore
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
        
        # Alta incertezza sulla velocità — ci fidiamo poco della predizione
        self.incertezza_modello = np.diag([0.01, 0.01, 0.5, 0.5]).astype(np.float32)
        self.incertezza_sensore = np.eye(2, dtype=np.float32) * 0.05
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
        self.soglia_perdita = 15

        # Ultima area valida del contorno. Serve a marcare come utilizzabili le
        # posizioni predette durante una perdita di segnale: `z` è il flag di
        # validità letto a valle, e ricopiare lo zero del messaggio in ingresso
        # le farebbe scartare come "bersaglio assente".
        self.ultima_area = 0.0

        self.get_logger().info('TrackerNode avviato — filtro Kalman attivo')

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
        r_base = 0.05
        r_max  = 2.0
        r_dinamico = r_base + (r_max - r_base) * self.livello_rumore
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
            @ self.evoluzione_stato.T + self.incertezza_modello
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

            # Smorza la velocità stimata per ridurre predizioni errate
            self.stato_stimato[2] *= 0.6
            self.stato_stimato[3] *= 0.6

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