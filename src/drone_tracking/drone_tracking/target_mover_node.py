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
        
        # Ritardo prima della fuga, in SECONDI. Contava messaggi di
        # /mission/stato, che arrivano a 2 Hz e non a 10: 50 conteggi facevano
        # 25 secondi invece dei 5 dichiarati. Ora si misura il tempo reale, così
        # il valore non dipende dalla frequenza del topic.
        self.istante_aggancio  = None
        self.ritardo_evasione_s = 10.0

        # I parametri di moto sono espressi in unità AL SECONDO e integrati sul
        # dt reale. Prima erano incrementi per tick, con il timer dichiarato a
        # 10 Hz: ma `gz service` impiega ~360 ms a rispondere e, chiamato in modo
        # bloccante, teneva il timer a ~2.8 Hz. I valori qui sotto riproducono la
        # velocità effettivamente osservata con quel timer strozzato.
        self.centro_x = 20.0
        self.centro_y = 20.0
        self.raggio   = 3.0
        self.velocita_angolare = 1.4    # rad/s  (era 0.5 rad/tick a ~2.8 Hz)

        # Parametri evasione
        self.vel_evasione      = 1.1    # m/s    (era 0.4 m/tick a ~2.8 Hz)
        self.dir_evasione_x    = 0.0
        self.dir_evasione_y    = 0.0
        self.tempo_evasione    = 0.0
        self.durata_evasione_s = 49.0   # s      (era 150 tick a ~2.8 Hz)

        self.ultimo_istante = None
        self.proc_pendente  = None

        self.timer = self.create_timer(0.1, self.muovi_bersaglio)
        self.get_logger().info('TargetMoverNode avviato — comportamento adattivo')

    def on_drone_pos(self, msg: PoseStamped):
        self.drone_x = msg.pose.position.x
        self.drone_y = msg.pose.position.y

    def on_stato_missione(self, msg: String):
        if FaseMissione.AGGANCIO.value in msg.data and self.fase == FaseBersaglio.PATTUGLIO:
            adesso = self.get_clock().now().nanoseconds / 1e9
            if self.istante_aggancio is None:
                self.istante_aggancio = adesso
            elif adesso - self.istante_aggancio >= self.ritardo_evasione_s:
                self.avvia_evasione()
        else:
            self.istante_aggancio = None

    def _calcola_dt(self):
        """Intervallo reale dall'ultimo ciclo, con clamp di sicurezza."""
        adesso = self.get_clock().now().nanoseconds / 1e9
        if self.ultimo_istante is None:
            self.ultimo_istante = adesso
            return 0.1
        dt = adesso - self.ultimo_istante
        self.ultimo_istante = adesso
        return float(min(max(dt, 0.01), 0.5))

    def avvia_evasione(self):
        self.fase = FaseBersaglio.EVASIONE
        self.tempo_evasione = 0.0
        self.istante_aggancio = None

        # Direzione di fuga — opposta al drone
        dx = self.pos_x - self.drone_x
        dy = self.pos_y - self.drone_y
        dist = math.sqrt(dx**2 + dy**2) + 0.001

        self.dir_evasione_x = dx / dist
        self.dir_evasione_y = dy / dist

        self.get_logger().warn(f'EVASIONE avviata.')

    def muovi_bersaglio(self):
        dt = self._calcola_dt()

        if self.fase == FaseBersaglio.PATTUGLIO:
            self.t += self.velocita_angolare * dt
            self.pos_x = self.centro_x + self.raggio * math.cos(self.t)
            self.pos_y = self.centro_y + self.raggio * math.sin(self.t)

        elif self.fase == FaseBersaglio.EVASIONE:
            self.tempo_evasione += dt
            self.pos_x += self.dir_evasione_x * self.vel_evasione * dt
            self.pos_y += self.dir_evasione_y * self.vel_evasione * dt

            if self.tempo_evasione >= self.durata_evasione_s:
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

        # La chiamata NON va attesa: `gz service` impiega ~360 ms e con
        # subprocess.run bloccava il timer a ~2.8 Hz invece dei 10 richiesti.
        # Si lancia in background e si controlla l'esito al ciclo successivo:
        # gli errori restano visibili, il timer resta libero.
        if self.proc_pendente is not None:
            rc = self.proc_pendente.poll()
            if rc is None:
                # Ancora in corso dopo un ciclo intero: si abbandona, tanto il
                # comando di posa che segue lo rende comunque superato.
                self.proc_pendente.kill()
                self.proc_pendente.wait()
            elif rc != 0:
                errore = self.proc_pendente.stderr.read().decode(errors='replace').strip()
                self.get_logger().warn(
                    f'gz service set_pose fallito (rc={rc}): {errore}',
                    throttle_duration_sec=5.0)

        self.proc_pendente = subprocess.Popen(
            cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)

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