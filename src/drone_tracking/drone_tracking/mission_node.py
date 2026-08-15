#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from enum import Enum
import math

class FaseMissione(Enum):
    ATTESA = "ATTESA"
    PATTUGLIAMENTO = "PATTUGLIAMENTO"
    AGGANCIO = "AGGANCIO"
    RICERCA = "RICERCA"


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        # Per la gestione pacchetti persi
        qos_mavros = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriber: posizione drone
        self.pos_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.on_position, qos_mavros)
        
        # Subscriber: Avvio di missione
        self.start_sub = self.create_subscription(
            Bool, '/mission/avvia',
            self.on_avvia, 10)
        
        # Subscriber: bersaglio rilevato
        self.target_sub = self.create_subscription(
            Point, '/target/tracked_position',
            self.on_target, 10)

        # Publisher: waypoint verso MAVROS2
        self.waypoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)

        # Publisher: stato missione
        self.stato_pub = self.create_publisher(
            String, '/mission/stato', 10)

        # Publisher: per resettare lo stato missione
        self.reset_pub = self.create_publisher(Bool, '/tracker/reset', 10)
        
        # Il drone va dritto verso la zona di oscillazione (8, 0)
        self.waypoints = [
            ( 0.0,  0.0, 12.0),   # origine - partenza
            (20.0,  0.0, 12.0),   # nord
            (20.0, 20.0, 12.0),   # nord-est - pallina qui
            ( 0.0, 20.0, 12.0),   # est
            ( 0.0,  0.0, 12.0),   # ritorno
        ]

        self.waypoint_corrente = 0
        self.posizione_attuale = None
        self.bersaglio_agganciato = False
        
        # Soglia a 1.2 metri
        self.soglia_waypoint = 1.2  
        
        self.rilevamento_attivo = False
        self.fase = FaseMissione.ATTESA

        self.frame_conferma_richiesti = 5
        self.frame_bersaglio_visibile = 0
        
        self.timer = self.create_timer(0.5, self.aggiorna_missione)
        self.get_logger().info('MissionNode avviato — in attesa di arming')
        
        # Variabili per la ricerca bersaglio se non agganciato
        self.ricerca_centro_x = 0.0
        self.ricerca_centro_y = 0.0
        self.ricerca_raggio   = 3.0
        self.ricerca_t        = 0.0
        self.ricerca_espansione = 0.0
        self.frame_senza_bersaglio = 0
        self.soglia_avvia_ricerca = 20  # frame (~2 sec) prima di avviare ricerca

    def on_position(self, msg: PoseStamped):
        self.posizione_attuale = msg.pose.position
        
        # self.get_logger().info(f'Pos: x:{self.posizione_attuale.x:.1f} y:{self.posizione_attuale.y:.1f}')

    def on_target(self, msg: Point):
        target_visibile = (msg.x != 0.0 or msg.y != 0.0)
        altitudine_ok = (self.posizione_attuale is not None
                        and self.posizione_attuale.z > 2.0)
        in_pattugliamento = self.fase == FaseMissione.PATTUGLIAMENTO

        # Gestione perdita bersaglio in fase AGGANCIO
        if self.fase == FaseMissione.AGGANCIO:
            if not target_visibile:
                self.frame_senza_bersaglio += 1
                if self.frame_senza_bersaglio > self.soglia_avvia_ricerca:
                    self.get_logger().warn('Bersaglio perso — avvio ricerca')
                    if self.posizione_attuale:
                        self.ricerca_centro_x = self.posizione_attuale.x
                        self.ricerca_centro_y = self.posizione_attuale.y
                    self.ricerca_t = 0.0
                    self.ricerca_espansione = 0.0
                    self.fase = FaseMissione.RICERCA
                    self.bersaglio_agganciato = False
            else:
                self.frame_senza_bersaglio = 0
            return

        # Gestione riaggancio in fase RICERCA
        if self.fase == FaseMissione.RICERCA:
            if target_visibile:
                self.frame_bersaglio_visibile += 1
                if self.frame_bersaglio_visibile >= self.frame_conferma_richiesti:
                    self.get_logger().warn('Bersaglio riagganciato')
                    self.bersaglio_agganciato = True
                    self.frame_senza_bersaglio = 0
                    self.fase = FaseMissione.AGGANCIO
            else:
                self.frame_bersaglio_visibile = 0
            return

        # Logica pattugliamento esistente
        if not (in_pattugliamento and altitudine_ok and self.rilevamento_attivo):
            self.frame_bersaglio_visibile = 0
            return

        if target_visibile:
            self.frame_bersaglio_visibile += 1
        else:
            self.frame_bersaglio_visibile = 0

        if (self.frame_bersaglio_visibile >= self.frame_conferma_richiesti
                and not self.bersaglio_agganciato):
            self.bersaglio_agganciato = True
            self.fase = FaseMissione.AGGANCIO
            self.get_logger().warn('Bersaglio agganciato')

    def distanza_waypoint(self, target):
        if self.posizione_attuale is None:
            return float('inf')
        dx = self.posizione_attuale.x - target[0]
        dy = self.posizione_attuale.y - target[1]
        dz = self.posizione_attuale.z - target[2]
        return (dx**2 + dy**2 + dz**2) ** 0.5

    def pubblica_waypoint(self, wp):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = wp[0]
        msg.pose.position.y = wp[1]
        msg.pose.position.z = wp[2]
        msg.pose.orientation.w = 1.0
        self.waypoint_pub.publish(msg)

    def aggiorna_missione(self):
        stato_msg = String()

        if self.fase == FaseMissione.ATTESA:
            stato_msg.data = FaseMissione.ATTESA.value
            self.stato_pub.publish(stato_msg)
            return

        if self.fase == FaseMissione.AGGANCIO:
            stato_msg.data = FaseMissione.AGGANCIO.value
            self.stato_pub.publish(stato_msg)
            return

        if self.fase == FaseMissione.RICERCA:
            stato_msg.data = FaseMissione.RICERCA.value
            self.stato_pub.publish(stato_msg)
            self.esegui_ricerca()
            return
        
        if self.waypoint_corrente >= len(self.waypoints):
            self.waypoint_corrente = 1  # Ricomincia il pattugliamento se non aggancia

        wp = self.waypoints[self.waypoint_corrente]
        self.pubblica_waypoint(wp)

        dist = self.distanza_waypoint(wp)
        self.get_logger().info(
            f'Waypoint {self.waypoint_corrente}/{len(self.waypoints)-1} '
            f'→ ({wp[0]:.0f},{wp[1]:.0f},{wp[2]:.0f})m '
            f'dist:{dist:.1f}m')

        if dist < self.soglia_waypoint:
            self.get_logger().info(f'Waypoint {self.waypoint_corrente} raggiunto vicino alla zona di oscillazione!')
            self.waypoint_corrente += 1

        stato_msg.data = f'{FaseMissione.PATTUGLIAMENTO.value}:{self.waypoint_corrente}'
        self.stato_pub.publish(stato_msg)

    def avvia_pattugliamento(self):
        self.fase = FaseMissione.PATTUGLIAMENTO
        self.waypoint_corrente = 0
        self.bersaglio_agganciato = False
        self.rilevamento_attivo = False
        
        reset_msg = Bool()
        reset_msg.data = True
        self.reset_pub.publish(reset_msg)
        
        # Timer per dare tempo al drone di decollare prima di attivare la visione
        self.crea_timer_visione = self.create_timer(2.0, self.abilita_rilevamento)
        self.get_logger().info('Pattugliamento avviato!')

    def abilita_rilevamento(self):
        self.rilevamento_attivo = True
        self.get_logger().info('Rilevamento bersaglio attivo')
        if hasattr(self, 'crea_timer_visione'):
            self.crea_timer_visione.destroy() # Distrugge il timer usa-e-getta per non accumulare callback
    
    def on_avvia(self, msg: Bool):
        self.get_logger().info(f'Ricevuto avvia: {msg.data} fase: {self.fase}')
        if msg.data and self.fase == FaseMissione.ATTESA:
            self.avvia_pattugliamento()

    def esegui_ricerca(self):
        # Spirale quadra espandibile intorno all'ultima posizione nota
        self.ricerca_t += 0.1
        self.ricerca_espansione += 0.002

        raggio_corrente = self.ricerca_raggio + self.ricerca_espansione

        x = self.ricerca_centro_x + raggio_corrente * math.cos(self.ricerca_t)
        y = self.ricerca_centro_y + raggio_corrente * math.sin(self.ricerca_t)
        z = self.waypoints[1][2]  # mantieni altitudine di crociera

        self.pubblica_waypoint((x, y, z))
        self.get_logger().info(
            f'RICERCA spirale -> ({x:.1f}, {y:.1f}) raggio:{raggio_corrente:.1f}m')

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()