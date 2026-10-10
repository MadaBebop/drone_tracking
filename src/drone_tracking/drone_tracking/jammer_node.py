#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import PointStamped
import numpy as np

from drone_tracking.parametri import parametro  # type: ignore

class JammerNode(Node):
    def __init__(self):
        super().__init__('jammer_node')

        # Publisher: stato del disturbo sul canale di PERCEZIONE. Non e il
        # GPS: quello lo attacca gnss_denial_node, su /gps/denial_active.
        self.disturbo_pub = self.create_publisher(
            Bool, '/percezione/disturbata', 10)

        # Publisher: intensità del rumore sul datalink (0.0 - 1.0)
        self.noise_pub = self.create_publisher(
            Float32, '/rf/noise_level', 10)

        # Subscriber: intercetta la posizione grezza e la corrompe
        self.sub = self.create_subscription(
            PointStamped, '/target/position', self.corrupt_signal, 10)

        # Publisher: posizione corrotta (quella che arriva al tracker sotto jamming)
        self.corrupted_pub = self.create_publisher(
            PointStamped, '/target/jammed_position', 10)

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

        # --- Falso aggancio: il guasto polarizzato ---
        # Probabilita che una finestra di disturbo, invece di sporcare la
        # misura, la sposti su un BERSAGLIO FALSO e ce la tenga. Zero disattiva
        # il modo e lascia il comportamento precedente.
        self.probabilita_falso_aggancio = parametro(
            self, 'probabilita_falso_aggancio', 0.0)
        self.durata_falso_aggancio_s = parametro(
            self, 'durata_falso_aggancio_s', 3.0)
        self.falso_fino_a = 0.0
        self.falso_posizione = None

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
            self.get_logger().warn('DISTURBO ATTIVO — rilevamento del bersaglio corrotto')
            self._forse_falso_aggancio()
        elif not self.jamming_active and was_jamming:
            self.get_logger().info('DISTURBO CESSATO — rilevamento pulito')

        # Pubblica stato GPS
        gps_msg = Bool()
        gps_msg.data = self.jamming_active
        self.disturbo_pub.publish(gps_msg)

        # Pubblica livello rumore RF
        noise_msg = Float32()
        noise_msg.data = 0.8 if self.jamming_active else 0.0
        self.noise_pub.publish(noise_msg)

    def _forse_falso_aggancio(self):
        """All inizio di una finestra di disturbo, decide se inventare un bersaglio.

        La posizione del falso resta FERMA per tutta la durata: e cio che lo
        distingue dal rumore. Un oggetto fisso nella scena non si sposta da un
        fotogramma all altro, e il filtro lo vede come un bersaglio
        perfettamente tracciabile che semplicemente non e il suo.
        """
        if self.probabilita_falso_aggancio <= 0.0:
            return
        if np.random.random() >= self.probabilita_falso_aggancio:
            return
        adesso = self.get_clock().now().nanoseconds / 1e9
        self.falso_fino_a = adesso + self.durata_falso_aggancio_s
        self.falso_posizione = (float(np.random.uniform(-0.8, 0.8)),
                                float(np.random.uniform(-0.8, 0.8)))
        self.get_logger().warn(
            'FALSO AGGANCIO a ({:.2f}, {:.2f}) per {:.0f} s'.format(
                self.falso_posizione[0], self.falso_posizione[1],
                self.durata_falso_aggancio_s))

    def _falso_attivo(self):
        if self.falso_posizione is None:
            return False
        if self.get_clock().now().nanoseconds / 1e9 >= self.falso_fino_a:
            self.falso_posizione = None
            return False
        return True

    def corrupt_signal(self, msg: PointStamped):
        # Limitiamo la frequenza di pubblicazione per evitare di sovraccaricare il tracker
        adesso = self.get_clock().now().nanoseconds / 1e9
        
        if adesso - self.ultimo_publish < self.throttle_hz:
            return
        
        self.ultimo_publish = adesso
        
        out = PointStamped()
        # Disturbare il canale cambia il valore, non l'istante in cui la scena
        # e stata guardata: l'header passa intatto.
        out.header = msg.header
        segnale_valido = not (msg.point.z == 0.0)

        # Il falso aggancio ha la precedenza sul rumore: e un guasto diverso,
        # non una sua versione piu forte. L area resta quella vera, perche un
        # oggetto della taglia sbagliata lo scarterebbe gia il rilevatore.
        if self._falso_attivo() and segnale_valido:
            out.point.x = self.falso_posizione[0]
            out.point.y = self.falso_posizione[1]
            out.point.z = msg.point.z
            self.corrupted_pub.publish(out)
            return

        if self.jamming_active and segnale_valido:
            noise_x = np.random.normal(0, self.deviazione_rumore) # rumore gaussiano con deviazione standard di 0.3
            noise_y = np.random.normal(0, self.deviazione_rumore)
            
            if np.random.random() < self.probabilita_perdita_segnale:
                out.point.x = 0.0
                out.point.y = 0.0
                out.point.z = 0.0
            else:
                out.point.x = float(np.clip(msg.point.x + noise_x, -1.0, 1.0))
                out.point.y = float(np.clip(msg.point.y + noise_y, -1.0, 1.0))
                out.point.z = msg.point.z
        else:
            out.point.x = msg.point.x
            out.point.y = msg.point.y
            out.point.z = msg.point.z
        
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