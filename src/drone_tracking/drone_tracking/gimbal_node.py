#!/usr/bin/env python3
"""Stabilizzazione della telecamera: comanda i due giunti della sospensione.

Il problema che risolve e geometrico, non di taratura. La telecamera era
solidale al corpo, e un multirotore accelera inclinandosi: ogni comando di
inseguimento produceva quindi una rotazione che traslava l'inquadratura
indipendentemente da dove fosse il bersaglio. Piu il controllo era pronto, piu
il velivolo si inclinava, e prima il bersaglio uscira dal campo — le due
grandezze non si possono ottimizzare separatamente.

La compensazione analitica in controller_node sottrae quella traslazione
dall'errore, e resta al suo posto: corregge il controllo, ma non
l'osservazione. Se l'inclinazione porta il bersaglio fuori dai pixel, nessun
calcolo lo recupera. Questo nodo agisce invece sulla causa, tenendo l'asse
ottico fermo rispetto al terreno mentre il corpo ruota.

Il comando e l'opposto dell'assetto misurato, saturato ai limiti del giunto:
oltre 45 gradi la sospensione entrerebbe nell'inquadratura, e da quel punto il
residuo torna a carico della compensazione analitica, che continua a esistere
proprio per questo.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

from drone_tracking.parametri import parametro  # type: ignore


class GimbalNode(Node):
    def __init__(self):
        super().__init__('gimbal_node')

        # Limite meccanico dei giunti, come dichiarato nel modello SDF: il nodo
        # non deve comandare oltre, altrimenti il regolatore inseguirebbe un
        # riferimento irraggiungibile accumulando errore.
        parametro(self, 'limite_rad', 0.7854)
        # Frazione dell'assetto da compensare. A 1.0 la stabilizzazione e
        # completa; valori inferiori servono a misurare quanto conta, senza
        # ricompilare.
        parametro(self, 'guadagno', 1.0)
        # A false il nodo comanda zero: la telecamera si comporta come se fosse
        # imbullonata al corpo. E il modo per ottenere la configurazione di
        # riferimento senza toccare il modello, quindi con lo stesso velivolo,
        # le stesse masse e la stessa dinamica.
        parametro(self, 'abilitato', True)
        parametro(self, 'frequenza_hz', 50.0)
        # Oltre questo tempo senza posa il comando resta all'ultimo valore, come
        # farebbe un servo reale, ma l'anomalia va segnalata: una telecamera
        # bloccata in una posizione qualsiasi e peggio di una fissa.
        parametro(self, 'timeout_posa_s', 1.0)

        qos_mavros = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.roll = 0.0
        self.pitch = 0.0
        self.istante_posa = None

        self.pub_roll = self.create_publisher(Float64, '/gimbal/roll/cmd_pos', 10)
        self.pub_pitch = self.create_publisher(Float64, '/gimbal/pitch/cmd_pos', 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self.on_posa, qos_mavros)

        self.timer = self.create_timer(1.0 / self.frequenza_hz, self.comanda)
        self.get_logger().info(
            'GimbalNode avviato — {}, guadagno {:.2f}, limite {:.0f}°'.format(
                'stabilizzazione attiva' if self.abilitato
                else 'DISATTIVATO (telecamera come fissa)',
                self.guadagno, math.degrees(self.limite_rad)))

    def on_posa(self, msg: PoseStamped):
        self.istante_posa = self.get_clock().now().nanoseconds / 1e9
        q = msg.pose.orientation
        # Stessa estrazione usata in controller_node: l'assetto va letto dalla
        # stessa sorgente che alimenta la compensazione analitica, altrimenti
        # le due correzioni lavorerebbero su misure diverse.
        self.pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        self.roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                               1.0 - 2.0 * (q.x * q.x + q.y * q.y))

    def _satura(self, valore):
        return max(-self.limite_rad, min(self.limite_rad, valore))

    def comanda(self):
        if not self.abilitato:
            self.pub_roll.publish(Float64(data=0.0))
            self.pub_pitch.publish(Float64(data=0.0))
            return

        adesso = self.get_clock().now().nanoseconds / 1e9
        if (self.istante_posa is None
                or adesso - self.istante_posa > self.timeout_posa_s):
            self.get_logger().error(
                'Nessuna posa da /mavros/local_position/pose: la telecamera '
                'resta all ultimo comando e non e piu stabilizzata',
                throttle_duration_sec=5.0)
            return

        # Segno opposto all'assetto: se il corpo rolla di +phi, il giunto deve
        # ruotare di -phi perche l'asse ottico resti dove era.
        self.pub_roll.publish(Float64(data=self._satura(-self.roll * self.guadagno)))
        self.pub_pitch.publish(Float64(data=self._satura(-self.pitch * self.guadagno)))


def main(args=None):
    rclpy.init(args=args)
    node = GimbalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
