#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool, Float64, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from drone_tracking.mission_node import FaseMissione  # type: ignore

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        qos_mavros = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.alt_sub = self.create_subscription(
            Float64, '/mavros/global_position/rel_alt',
            self.on_altitudine, qos_mavros)

        self.mission_sub = self.create_subscription(
            String, '/mission/stato',
            self.on_stato_missione, 10)

        self.sub = self.create_subscription(
            Point, '/target/tracked_position', self.on_tracked, 10)

        self.gps_sub = self.create_subscription(
            Bool, '/gps/jammed', self.on_gps_status, 10)

        self.cmd_pub = self.create_publisher(
            Twist, '/drone/cmd_vel', 10)

        self.mavros_vel_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        self.gps_jammed     = False
        self.altitudine     = 0.0
        self.in_volo        = False
        self.fase_missione  = FaseMissione.ATTESA.value

        # Guadagni PD
        self.kp_x = 4.0
        self.kp_y = 4.0
        self.kd_x = 0.8
        self.kd_y = 0.8
        self.vel_max = 8.0
        
        self.deadzone = 0.05
        self.altitudine_crociera = 12.0

        self.error_x_prev      = 0.0
        self.error_y_prev      = 0.0
        self.primo_aggancio    = True
        self.cmd_corrente      = Twist()

        self.vel_timer = self.create_timer(0.1, self.pubblica_velocita_continua)
        self.get_logger().info('ControllerNode avviato')

    def pubblica_velocita_continua(self):
        fase_ok = FaseMissione.AGGANCIO.value in self.fase_missione
        if self.in_volo and fase_ok:
            self.mavros_vel_pub.publish(self.cmd_corrente)

    def on_stato_missione(self, msg: String):
        self.fase_missione = msg.data
        # Reset aggancio quando si esce da AGGANCIO
        if FaseMissione.AGGANCIO.value not in msg.data:
            self.primo_aggancio = True
            self.cmd_corrente = Twist()

    def on_gps_status(self, msg: Bool):
        if msg.data and not self.gps_jammed:
            self.get_logger().warn('GPS perso — modalità visione attiva')
        elif not msg.data and self.gps_jammed:
            self.get_logger().info('GPS ripristinato')
        self.gps_jammed = msg.data

    def on_tracked(self, msg: Point):
        cmd = Twist()
        target_visible = (msg.x != 0.0 or msg.y != 0.0)

        if not target_visible:
            self.cmd_corrente = Twist()
            self.primo_aggancio = True
            return

        # Guardia FOV — ignora predizioni Kalman fuori range
        if abs(msg.x) > 1.2 or abs(msg.y) > 1.2:
            self.cmd_corrente = Twist()
            return

        error_x = msg.x
        error_y = msg.y

        # Evita derivative kick al primo frame
        if self.primo_aggancio:
            self.error_x_prev = error_x
            self.error_y_prev = error_y
            self.primo_aggancio = False

        deriv_x = (error_x - self.error_x_prev) / 0.1
        deriv_y = (error_y - self.error_y_prev) / 0.1
        self.error_x_prev = error_x
        self.error_y_prev = error_y

        scala = max(self.altitudine, 1.0) / self.altitudine_crociera
        kp_x  = self.kp_x * scala
        kp_y  = self.kp_y * scala

        if abs(error_x) > self.deadzone:
            cmd.linear.y = -(kp_x * error_x + self.kd_x * deriv_x)
            cmd.linear.y = max(-self.vel_max, min(self.vel_max, cmd.linear.y))

        if abs(error_y) > self.deadzone:
            cmd.linear.x = -(kp_y * error_y + self.kd_y * deriv_y)
            cmd.linear.x = max(-self.vel_max, min(self.vel_max, cmd.linear.x))

        self.cmd_corrente = cmd
        self.cmd_pub.publish(cmd)

        mode = 'GPS+VISIONE' if not self.gps_jammed else 'SOLO VISIONE'
        # self.get_logger().info(
        #     f'[{mode}] err:({error_x:.2f},{error_y:.2f}) '
        #     f'→ v:({cmd.linear.y:.2f},{cmd.linear.x:.2f})')

    def on_altitudine(self, msg):
        self.altitudine = msg.data
        self.in_volo    = self.altitudine > 1.0

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()