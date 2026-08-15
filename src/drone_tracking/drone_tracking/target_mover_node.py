#!/usr/bin/env python3
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import subprocess
import math
from drone_tracking.mission_node import FaseMissione  # type: ignore

class FaseBersaglio(Enum):
    PATTUGLIO = "PATTUGLIO"
    EVASIONE  = "EVASIONE"
    
class TargetMoverNode(Node):
    def __init__(self):
        super().__init__('target_mover_node')

        qos_mavros = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber: stato missione — triggera evasione su AGGANCIO
        self.mission_sub = self.create_subscription(
            String, '/mission/stato',
            self.on_stato_missione, 10)

        # Subscriber: posizione drone — per evasione intelligente
        self.drone_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.on_drone_pos, qos_mavros)

        # Stato interno
        self.fase = FaseBersaglio.PATTUGLIO
        self.t = 0.0
        self.pos_x = 20.0
        self.pos_y = 20.0
        self.drone_x = 0.0
        self.drone_y = 0.0
        
        self.ritardo_evasione = 0
        self.durata_ritardo_evasione = 50  # 5 secondi prima di scappare

        # Parametri pattugliamento circolare
        self.centro_x  = 20.0
        self.centro_y  = 20.0
        self.raggio    = 3.0
        self.velocita  = 0.5   # 1.5 m/s normale

        # Parametri evasione
        self.vel_evasione   = 0.4    # 12 m/s — veicolo in fuga
        self.dir_evasione_x = 0.0
        self.dir_evasione_y = 0.0
        self.evasione_timer = 0
        self.durata_evasione = 150   # tick (15 sec) prima di ricominciare

        self.timer = self.create_timer(0.1, self.muovi_bersaglio)
        self.get_logger().info('TargetMoverNode avviato — comportamento adattivo')

    def on_drone_pos(self, msg: PoseStamped):
        self.drone_x = msg.pose.position.x
        self.drone_y = msg.pose.position.y

    def on_stato_missione(self, msg: String):
        if FaseMissione.AGGANCIO.value in msg.data and self.fase == FaseBersaglio.PATTUGLIO:
            self.ritardo_evasione += 1
            if self.ritardo_evasione >= self.durata_ritardo_evasione:
                self.avvia_evasione()
        else:
            self.ritardo_evasione = 0

    def avvia_evasione(self):
        self.fase = FaseBersaglio.EVASIONE
        self.evasione_timer = 0

        # Direzione di fuga — opposta al drone
        dx = self.pos_x - self.drone_x
        dy = self.pos_y - self.drone_y
        dist = math.sqrt(dx**2 + dy**2) + 0.001

        self.dir_evasione_x = dx / dist
        self.dir_evasione_y = dy / dist

        self.get_logger().warn(f'EVASIONE avviata.')

    def muovi_bersaglio(self):
        if self.fase == FaseBersaglio.PATTUGLIO:
            self.t += self.velocita
            self.pos_x = self.centro_x + self.raggio * math.cos(self.t)
            self.pos_y = self.centro_y + self.raggio * math.sin(self.t)

        elif self.fase == FaseBersaglio.EVASIONE:
            self.evasione_timer += 1
            self.pos_x += self.dir_evasione_x * self.vel_evasione
            self.pos_y += self.dir_evasione_y * self.vel_evasione

            # Dopo durata_evasione tick torna al pattugliamento
            if self.evasione_timer >= self.durata_evasione:
                self.fase = FaseBersaglio.PATTUGLIO
                self.centro_x = self.pos_x
                self.centro_y = self.pos_y
                self.t = 0.0
                self.get_logger().info('Evasione completata — riprende pattugliamento')

        z = 0.5
        req = (f'name: "bersaglio" '
               f'position: {{x: {self.pos_x:.3f}, y: {self.pos_y:.3f}, z: {z:.3f}}} '
               f'orientation: {{w: 1.0}}')
        cmd = ['gz', 'service', '-s', '/world/iris_runway/set_pose',
               '--reqtype', 'gz.msgs.Pose',
               '--reptype', 'gz.msgs.Boolean',
               '--timeout', '500', '--req', req]
        subprocess.run(cmd, capture_output=True)

        # self.get_logger().info(
        #     f'[{self.fase}] Bersaglio -> ({self.pos_x:.1f}, {self.pos_y:.1f})')

def main(args=None):
    rclpy.init(args=args)
    node = TargetMoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()