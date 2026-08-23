#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, Twist
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

        # Serve lo yaw per convertire i comandi dal frame del drone a quello del
        # mondo: vedi la nota in on_tracked.
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.on_posa, qos_mavros)

        self.cmd_pub = self.create_publisher(
            Twist, '/drone/cmd_vel', 10)

        self.mavros_vel_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        self.gps_jammed     = False
        self.altitudine     = 0.0
        self.in_volo        = False
        self.fase_missione  = FaseMissione.ATTESA.value
        self.yaw            = 0.0
        self.pitch          = 0.0
        self.roll           = 0.0

        # Compensazione d'assetto. La telecamera è solidale al corpo: una
        # rotazione del velivolo trasla l'immagine indipendentemente da dove si
        # trovi il bersaglio. Con FOV orizzontale 1.047 rad su 640x480, i
        # semicampi valgono 0.524 rad in orizzontale e 0.408 in verticale, quindi
        # un radiante di assetto vale 1/0.408 ≈ 2.45 unità normalizzate: bastano
        # 10° di pitch per spostare il bersaglio di mezzo campo.
        # Misurato senza compensazione: correlazione r = -0.665 fra pitch ed
        # errore verticale, con l'errore che spazzava l'intero campo visivo
        # mentre il bersaglio era pressoché fermo.

        # Guadagni PD espressi in 1/s: agiscono sullo scostamento del bersaglio
        # in METRI, non sulle coordinate normalizzate dell'immagine. Quelle
        # cambiano significato al variare di quota e campo visivo — lo stesso
        # 0.3 vale 2 m a 12 m di quota con FOV 60° e 3.6 m con FOV 90° — quindi
        # una taratura fatta su di esse va rifatta a ogni modifica dell'ottica.
        # In metri il guadagno ha un senso fisico diretto: kp = 1.2 significa
        # 1.2 m/s di comando per ogni metro di scarto, cioe uno scarto a regime
        # di circa velocita_bersaglio / kp = 1 m contro un bersaglio a 1.2 m/s.
        # Provato a scendere a 0.6 con smorzamento 0.6, nell'ipotesi che l'anello
        # oscillasse: distanza mediana dal bersaglio da 3.5 a 26.7 m e tempo in
        # AGGANCIO dal 100% al 28%. Con guadagno basso il drone non tiene il
        # passo. 1.2 resta il valore migliore misurato.
        self.kp_x = 1.2      # 1/s
        self.kp_y = 1.2
        self.kd_x = 0.35
        self.kd_y = 0.35
        # 8 m/s erano molti piu del necessario: il bersaglio si muove intorno a
        # 1 m/s, quindi quel tetto non veniva mai raggiunto per inseguirlo ma
        # solo durante i transitori, dove produceva sorpassi e oscillazione.
        self.vel_max = 3.5

        self.deadzone = 0.3          # metri
        self.semi_fov_o = 0.7854     # rad, meta del FOV orizzontale (90°)
        self.semi_fov_v = 0.6435     # rad, meta del FOV verticale su 640x480

        self.error_x_prev      = 0.0
        self.error_y_prev      = 0.0
        self.primo_aggancio    = True
        self.cmd_corrente      = Twist()

        # on_tracked è guidato dai messaggi del tracker, il cui ritmo segue la
        # telecamera (misurato fra 5 e 13 Hz): il termine derivativo va diviso
        # per l'intervallo reale, non per una costante.
        self.dt_nominale    = 0.1
        self.dt_min         = 0.02
        self.dt_max         = 0.5
        self.ultimo_istante = None

        self.vel_timer = self.create_timer(0.1, self.pubblica_velocita_continua)
        self.get_logger().info('ControllerNode avviato')

    def _calcola_dt(self):
        """Intervallo reale dall'ultima stima ricevuta, con clamp di sicurezza."""
        adesso = self.get_clock().now().nanoseconds / 1e9
        if self.ultimo_istante is None:
            self.ultimo_istante = adesso
            return self.dt_nominale
        dt = adesso - self.ultimo_istante
        self.ultimo_istante = adesso
        return float(min(max(dt, self.dt_min), self.dt_max))

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

    def on_posa(self, msg: PoseStamped):
        q = msg.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        self.roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                               1.0 - 2.0 * (q.x * q.x + q.y * q.y))

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
            self.ultimo_istante = None
            return

        # Guardia FOV — ignora predizioni Kalman fuori range
        if abs(msg.x) > 1.2 or abs(msg.y) > 1.2:
            self.cmd_corrente = Twist()
            return

        # Si sottrae la traslazione d'immagine dovuta all'assetto, così l'errore
        # rappresenta la posizione del bersaglio e non l'inclinazione del drone.
        norm_x = msg.x - self.roll / self.semi_fov_o
        norm_y = msg.y + self.pitch / self.semi_fov_v

        # Conversione in metri sul terreno: con la telecamera a nadir e quota h,
        # il semicampo copre h*tan(semi_fov), quindi una coordinata normalizzata
        # vale quella distanza per unità.
        quota = max(self.altitudine, 1.0)
        error_x = norm_x * quota * math.tan(self.semi_fov_o)
        error_y = norm_y * quota * math.tan(self.semi_fov_v)

        dt = self._calcola_dt()

        # Evita derivative kick al primo frame
        if self.primo_aggancio:
            self.error_x_prev = error_x
            self.error_y_prev = error_y
            self.primo_aggancio = False

        deriv_x = (error_x - self.error_x_prev) / dt
        deriv_y = (error_y - self.error_y_prev) / dt
        self.error_x_prev = error_x
        self.error_y_prev = error_y

        # Nessuna scalatura con la quota: l'errore è già in metri, e la quota è
        # entrata nella conversione da coordinate immagine a distanza al suolo.
        # Comando nel frame del drone.
        v_avanti = 0.0
        v_laterale = 0.0

        if abs(error_x) > self.deadzone:
            v_laterale = -(self.kp_x * error_x + self.kd_x * deriv_x)
            v_laterale = max(-self.vel_max, min(self.vel_max, v_laterale))

        if abs(error_y) > self.deadzone:
            v_avanti = -(self.kp_y * error_y + self.kd_y * deriv_y)
            v_avanti = max(-self.vel_max, min(self.vel_max, v_avanti))

        # Rotazione dal frame del drone a quello locale ENU.
        # `/mavros/setpoint_velocity/cmd_vel_unstamped` viene tradotto da MAVROS
        # in SET_POSITION_TARGET_LOCAL_NED con frame LOCAL_NED, cioè il frame del
        # MONDO: pubblicare lì un vettore calcolato nel frame della telecamera è
        # corretto solo se lo yaw è zero. In volo lo yaw non è controllato e
        # deriva: misurato 25° di media con ±21° di oscillazione, che portava il
        # comando a puntare in media 61° fuori bersaglio — il drone spingeva di
        # traverso e non riusciva a seguire nemmeno un'orbita lenta.
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        cmd.linear.x = v_avanti * cos_y - v_laterale * sin_y
        cmd.linear.y = v_avanti * sin_y + v_laterale * cos_y

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