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

# I binding Python di gz-transport riusano un nodo persistente: una richiesta
# costa ~0.4 ms, contro i ~360 ms del comando `gz service`, che a 10 Hz non
# starebbe mai dentro il periodo del timer. Se non sono installati si ricade sul
# CLI, con l'avvertenza stampata all'avvio.
try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_pb2 import Pose as GzPose
    from gz.msgs10.boolean_pb2 import Boolean as GzBoolean
    GZ_BINDINGS = True
except ImportError:
    GZ_BINDINGS = False

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
        # 0.35 rad/s su raggio 3 m = ~1.05 m/s tangenziali, passo d'uomo.
        # Era 1.4 rad/s, cioè 4.2 m/s: su un cerchio così stretto la direzione
        # si invertiva ogni due secondi e il drone non riusciva mai a
        # stabilizzarsi sopra il bersaglio.
        self.velocita_angolare = 0.35   # rad/s

        # Parametri evasione. Il limite non è la velocità massima del drone
        # (8 m/s) ma l'errore a regime del controllo proporzionale, che vale
        # circa `velocita_bersaglio / kp`: con kp = 4.0, a 2 m/s il bersaglio si
        # stabilizzava a 0.5 in coordinate normalizzate, cioè a metà del
        # semicampo visivo, e la prima finestra di jamming lo faceva uscire.
        # A 1.2 m/s l'errore a regime scende a ~0.3 e il margine regge.
        self.vel_evasione      = 1.2    # m/s
        self.dir_evasione_x    = 0.0
        self.dir_evasione_y    = 0.0
        self.tempo_evasione    = 0.0
        self.durata_evasione_s = 20.0   # s  -> ~40 m di fuga

        self.ultimo_istante = None
        self.proc_pendente  = None

        self.nome_modello  = 'bersaglio'
        self.quota_modello = 0.3   # raggio della sfera: la appoggia a terra
        self.servizio_posa = '/world/iris_runway/set_pose'

        if GZ_BINDINGS:
            self.gz_node = GzNode()
        else:
            self.gz_node = None
            self.get_logger().warn(
                'python3-gz-transport13 non disponibile: si usa il comando '
                '`gz service`, che costa ~360 ms a chiamata e limita '
                'l\'aggiornamento della posa a ~3 Hz. Il moto resta corretto '
                'perché integrato sul dt reale, ma meno fluido.')

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

        self._invia_posa(self.pos_x, self.pos_y, self.quota_modello)

        # self.get_logger().info(
        #     f'[{self.fase}] Bersaglio -> ({self.pos_x:.1f}, {self.pos_y:.1f})')

    def _invia_posa(self, x, y, z):
        """Comanda la posa del bersaglio nel simulatore."""
        if self.gz_node is not None:
            req = GzPose()
            req.name = self.nome_modello
            req.position.x = float(x)
            req.position.y = float(y)
            req.position.z = float(z)
            req.orientation.w = 1.0
            esito, risposta = self.gz_node.request(
                self.servizio_posa, req, GzPose, GzBoolean, 200)
            if not esito or not risposta.data:
                self.get_logger().warn(
                    f'set_pose rifiutato dal simulatore (esito={esito}). '
                    f'Il modello "{self.nome_modello}" esiste nel mondo?',
                    throttle_duration_sec=5.0)
            return

        # Fallback CLI. Il processo non viene atteso né ucciso: durando ~360 ms
        # verrebbe interrotto a ogni ciclo del timer prima di completare, e la
        # posa non arriverebbe mai al simulatore.
        if self.proc_pendente is not None:
            if self.proc_pendente.poll() is None:
                return
            if self.proc_pendente.returncode != 0:
                errore = self.proc_pendente.stderr.read().decode(errors='replace').strip()
                self.get_logger().warn(
                    f'gz service set_pose fallito: {errore}',
                    throttle_duration_sec=5.0)

        req = (f'name: "{self.nome_modello}" '
               f'position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}} '
               f'orientation: {{w: 1.0}}')
        self.proc_pendente = subprocess.Popen(
            ['gz', 'service', '-s', self.servizio_posa,
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '500', '--req', req],
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)

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