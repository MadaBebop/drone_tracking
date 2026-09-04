#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Point
import numpy as np

from drone_tracking.parametri import parametro  # type: ignore

class JammerNode(Node):
    def __init__(self):
        super().__init__('jammer_node')

        # Publisher: stato del jamming GPS
        self.gps_jammed_pub = self.create_publisher(
            Bool, '/gps/jammed', 10)

        # Publisher: intensità del rumore sul datalink (0.0 - 1.0)
        self.noise_pub = self.create_publisher(
            Float32, '/rf/noise_level', 10)

        # Subscriber: intercetta la posizione grezza e la corrompe
        self.sub = self.create_subscription(
            Point, '/target/position', self.corrupt_signal, 10)

        # Publisher: posizione corrotta (quella che arriva al tracker sotto jamming)
        self.corrupted_pub = self.create_publisher(
            Point, '/target/jammed_position', 10)

        # Stato interno
        self.jamming_active = False
        self.jam_cycle = 0
        self.jam_on_duration = parametro(
            self, 'jam_on_duration', 40)     # cicli attivi   (~4 s)
        self.jam_off_duration = parametro(
            self, 'jam_off_duration', 100)   # cicli inattivi (~10 s)

        self.ultimo_publish = 0.0
        self.throttle_hz = 0.05  # pubblica max ogni 50ms (per problema di sovraccarico del tracker)

        # 30% di probabilità di perdita totale del rilevamento per messaggio
        self.probabilita_perdita_segnale = parametro(
            self, 'probabilita_perdita_segnale', 0.3)
        # deviazione standard del rumore gaussiano, in coordinate normalizzate
        self.deviazione_rumore = parametro(self, 'deviazione_rumore', 0.3)

        # Seme del generatore pseudocasuale. Senza, ogni esecuzione vedeva una
        # sequenza di disturbo diversa: due prove della stessa configurazione
        # davano risultati che non si potevano confrontare, perché a cambiare
        # non era solo il codice ma anche lo stimolo. Fissandolo, il disturbo
        # diventa parte riproducibile dell'esperimento; per esplorare piu
        # sequenze si passa un valore diverso dalla riga di comando:
        #   ros2 run drone_tracking jammer_node --ros-args -p seed:=7
        # Un valore negativo torna al comportamento non deterministico.
        self.seed = int(parametro(self, 'seed', 42))   # cambiarlo a caldo non ha senso
        if self.seed >= 0:
            np.random.seed(self.seed)
            self.get_logger().info(f'Sequenza di disturbo deterministica — seed={self.seed}')
        else:
            self.get_logger().warn('seed negativo — disturbo non riproducibile fra run')

        # Timer principale — cicla jamming ON/OFF
        self.timer = self.create_timer(0.1, self.update)
        self.get_logger().info('JammerNode avviato — ciclo jamming ON/OFF automatico')

    def update(self):
        self.jam_cycle += 1

        total = self.jam_on_duration + self.jam_off_duration
        phase = self.jam_cycle % total

        was_jamming = self.jamming_active
        self.jamming_active = phase < self.jam_on_duration

        if self.jamming_active and not was_jamming:
            self.get_logger().warn('JAMMING ATTIVO — GPS e datalink disturbati')
        elif not self.jamming_active and was_jamming:
            self.get_logger().info('JAMMING DISATTIVO — segnali ripristinati')

        # Pubblica stato GPS
        gps_msg = Bool()
        gps_msg.data = self.jamming_active
        self.gps_jammed_pub.publish(gps_msg)

        # Pubblica livello rumore RF
        noise_msg = Float32()
        noise_msg.data = 0.8 if self.jamming_active else 0.0
        self.noise_pub.publish(noise_msg)

    def corrupt_signal(self, msg: Point):
        # Limitiamo la frequenza di pubblicazione per evitare di sovraccaricare il tracker
        adesso = self.get_clock().now().nanoseconds / 1e9
        
        if adesso - self.ultimo_publish < self.throttle_hz:
            return
        
        self.ultimo_publish = adesso
        
        out = Point()
        segnale_valido = not (msg.z == 0.0)

        if self.jamming_active and segnale_valido:
            noise_x = np.random.normal(0, self.deviazione_rumore) # rumore gaussiano con deviazione standard di 0.3
            noise_y = np.random.normal(0, self.deviazione_rumore)
            
            if np.random.random() < self.probabilita_perdita_segnale:
                out.x = 0.0
                out.y = 0.0
                out.z = 0.0
            else:
                out.x = float(np.clip(msg.x + noise_x, -1.0, 1.0))
                out.y = float(np.clip(msg.y + noise_y, -1.0, 1.0))
                out.z = msg.z
        else:
            out.x = msg.x
            out.y = msg.y
            out.z = msg.z
        
        self.corrupted_pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = JammerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()