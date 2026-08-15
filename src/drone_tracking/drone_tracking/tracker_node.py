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

        dt = 0.1

        self.evoluzione_stato = np.array([
            [1, 0, dt, 0 ],
            [0, 1, 0,  dt],
            [0, 0, 1,  0 ],
            [0, 0, 0,  1 ]
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
        self.soglia_perdita = 5

        self.get_logger().info('TrackerNode avviato — filtro Kalman attivo')

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

    def on_detection(self, msg: Point):
        segnale_presente = not (msg.z == 0.0)

        if not self.bersaglio_acquisito and segnale_presente:
            self.stato_stimato = np.array(
                [[msg.x], [msg.y], [0.0], [0.0]], dtype=np.float32)
            self.bersaglio_acquisito = True
            self.get_logger().info('Bersaglio acquisito')
            return

        if not self.bersaglio_acquisito:
            # Continua a pubblicare coordinate nulle se non ha ancora agganciato nulla
            msg_vuoto = Point(x=0.0, y=0.0, z=0.0)
            self.pub.publish(msg_vuoto)
            return

        # PREDIZIONE KALMAN
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
            
            # Pubblica la predizione per tollerare micro-interruzioni
            posizione_stimata = Point()
            posizione_stimata.x = float(self.stato_stimato[0].item())
            posizione_stimata.y = float(self.stato_stimato[1].item())
            posizione_stimata.z = float(msg.z)
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