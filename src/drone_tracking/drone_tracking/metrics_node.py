#!/usr/bin/env python3
"""Registrazione delle metriche di una prova su file CSV.

Una riga per campione, un file per prova, a frequenza fissa: cosi il file si
media e si diagramma senza reinterpolare.

La posizione «vera» di drone e bersaglio viene letta dal simulatore
(/world/<mondo>/pose/info) e non da MAVROS, perche la stima dell'EKF e essa
stessa oggetto di misura e non puo fare da riferimento a se stessa. Sono
registrate entrambe, cosi lo scarto fra le due resta visibile nei dati invece
di restare nascosto in un'assunzione.
"""
import csv
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from std_msgs.msg import Bool, Float32, Float64, String

# Binding nativi di gz-transport: costano una frazione di millisecondo contro
# le centinaia del CLI. Senza, il nodo registra solo cio che arriva da ROS 2.
try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_v_pb2 import Pose_V
    GZ_BINDINGS = True
except ImportError:
    GZ_BINDINGS = False

COLONNE = [
    't_sim', 't_wall', 'fase',
    'det_valido', 'det_x', 'det_y', 'det_area', 'det_hz',
    'jam_valido', 'jam_x', 'jam_y', 'rumore_rf', 'nis',
    'trk_valido', 'trk_x', 'trk_y', 'trk_area', 'trk_hz',
    'trk_vx', 'trk_vy',
    'ekf_x', 'ekf_y', 'ekf_z', 'roll', 'pitch', 'yaw',
    'vel_drone_x', 'vel_drone_y',
    'gimbal_roll', 'gimbal_pitch',
    'gt_drone_x', 'gt_drone_y', 'gt_drone_z',
    'gt_target_x', 'gt_target_y', 'gt_target_z',
    'dist_xy_gt', 'dist_3d_gt', 'dist_xy_ekf',
    'jam_attivo', 'gps_negato',
]

VUOTA = ['', '', '']


class MetricsNode(Node):
    def __init__(self):
        super().__init__('metrics_node')

        self.declare_parameter('cartella_output', '/ws/metrics')
        self.declare_parameter('etichetta_config', '')
        self.declare_parameter('frequenza_hz', 5.0)
        self.declare_parameter('mondo', 'iris_runway')
        self.declare_parameter('modello_drone', 'iris_with_ardupilot')
        self.declare_parameter('modello_bersaglio', 'bersaglio')

        self.etichetta = str(self.get_parameter('etichetta_config').value).strip()
        self.frequenza = float(self.get_parameter('frequenza_hz').value)
        self.mondo = str(self.get_parameter('mondo').value)
        self.nome_drone = str(self.get_parameter('modello_drone').value)
        self.nome_bersaglio = str(self.get_parameter('modello_bersaglio').value)

        qos_mavros = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(String, '/mission/stato', self.on_stato, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self.on_posa, qos_mavros)
        self.create_subscription(Point, '/target/position', self.on_detection, 10)
        # Ingresso vero del filtro, cioe il rilevamento dopo il jammer: e il
        # termine di paragone corretto per misurarne il guadagno.
        self.create_subscription(Point, '/target/jammed_position',
                                 self.on_disturbata, 10)
        # Livello di rumore dichiarato sul datalink: governa R nel tracker.
        self.create_subscription(Float32, '/rf/noise_level', self.on_rumore, 10)
        self.create_subscription(Float32, '/target/nis', self.on_nis, 10)
        self.create_subscription(Point, '/target/tracked_position', self.on_tracked, 10)
        # Velocita stimata dal filtro, in coordinate immagine al secondo. Da
        # confrontare con il moto vero del bersaglio per giudicarne la qualita.
        self.create_subscription(Point, '/target/tracked_velocity',
                                 self.on_tracked_vel, 10)
        # Velocita del velivolo. Registrarla accanto alla posizione vera
        # permette di stabilire in che frame sia espressa invece di assumerlo.
        self.create_subscription(TwistStamped,
                                 '/mavros/local_position/velocity_local',
                                 self.on_velocita_drone, qos_mavros)
        self.create_subscription(Bool, '/gps/jammed', self.on_jam, 10)
        # Si registra il COMANDO al giunto e non il suo angolo effettivo: e il
        # comando a dire cosa il nodo ha chiesto, e un segno sbagliato si
        # vedrebbe qui accanto alla colonna dell'assetto.
        self.create_subscription(Float64, '/gimbal/roll/cmd_pos',
                                 self.on_gimbal_roll, 10)
        self.create_subscription(Float64, '/gimbal/pitch/cmd_pos',
                                 self.on_gimbal_pitch, 10)
        # Pubblicato da gnss_denial_node; la colonna resta a 0 se non gira.
        self.create_subscription(Bool, '/gps/denial_active', self.on_denial, 10)

        # Ultimo valore visto per ciascuna sorgente: il campionamento e a
        # frequenza fissa, indipendente dall'arrivo dei messaggi.
        self.fase = 'ATTESA'
        self.ekf = None
        self.det = None
        # Non `self.jam`: quel nome e gia dello stato del jammer GPS.
        self.det_disturbata = None
        self.rumore_rf = None
        self.nis = None
        self.trk = None
        self.trk_vel = None
        self.jam = False
        self.gps_negato = False
        # Assetto del corpo: senza, l'effetto della stabilizzazione non si misura.
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.vel_drone = None
        self.gimbal_roll = None
        self.gimbal_pitch = None

        # Ritmo effettivo della catena di percezione, azzerato a ogni riga: la
        # telecamera gira fra 5 e 25 Hz secondo il carico.
        self.n_det = 0
        self.n_trk = 0

        self.gt_drone = None
        self.gt_bersaglio = None
        self.ultima_posa_gz = 0.0
        self.periodo_posa_gz = 0.1   # non serve leggere pose/info piu di 10 Hz

        # Statistiche cumulative per il riepilogo finale a schermo.
        self.n_campioni = 0
        self.n_visibili = 0
        self.somma_dist = 0.0
        self.n_dist = 0
        self.tempo_per_fase = {}
        self.istante_cambio_fase = None

        self.percorso = self._apri_csv()
        self.t0_wall = time.time()

        self.gz_node = None
        if GZ_BINDINGS:
            topic = '/world/{}/pose/info'.format(self.mondo)
            # La firma dei binding cambia fra le versioni, e un errore qui non
            # deve impedire la registrazione di cio che arriva da ROS 2.
            try:
                self.gz_node = GzNode()
                esito = self.gz_node.subscribe(Pose_V, topic, self.on_pose_info)
            except Exception as e:            # noqa: BLE001 - si vuole degradare
                self.get_logger().warn(
                    'sottoscrizione a {} non riuscita ({}): colonne gt_* '
                    'vuote'.format(topic, e))
                self.gz_node = None
                esito = False
            if esito:
                self.get_logger().info('Verita a terra da {}'.format(topic))
            elif self.gz_node is not None:
                self.get_logger().warn(
                    'sottoscrizione a {} rifiutata: colonne gt_* vuote'.format(topic))
                self.gz_node = None
        else:
            self.get_logger().warn(
                'python3-gz-transport13 non disponibile: nessuna verita a '
                'terra, le colonne gt_* e dist_*_gt restano vuote.')

        self.timer = self.create_timer(1.0 / self.frequenza, self.campiona)
        self.get_logger().info(
            'MetricsNode avviato a {:.1f} Hz -> {}'.format(self.frequenza, self.percorso))

    # ---------------------------------------------------------------- output

    def _apri_csv(self):
        cartella = str(self.get_parameter('cartella_output').value)
        # L'etichetta si rilegge a ogni apertura: cosi `ros2 param set` prima
        # dell'avvio da il nome alla prova senza rilanciare i nodi.
        self.etichetta = str(self.get_parameter('etichetta_config').value).strip()
        try:
            os.makedirs(cartella, exist_ok=True)
            prova = os.path.join(cartella, '.scrivibile')
            with open(prova, 'w'):
                pass
            os.remove(prova)
        except OSError as e:
            self.get_logger().warn(
                '{} non utilizzabile ({}): si scrive nella cartella corrente '
                '{}'.format(cartella, e, os.getcwd()))
            cartella = os.getcwd()

        marca = datetime.now().strftime('%Y%m%d_%H%M%S')
        nome = 'metrics_{}.csv'.format(marca) if not self.etichetta \
            else 'metrics_{}_{}.csv'.format(marca, self.etichetta)
        percorso = os.path.join(cartella, nome)
        # Due prove avviate nello stesso secondo sono improbabili ma non
        # impossibili, e sovrascrivere dati di misura non si fa.
        n = 1
        while os.path.exists(percorso):
            n += 1
            percorso = os.path.join(cartella, nome[:-4] + '_%d.csv' % n)

        # Riga per riga con flush: una prova interrotta a meta lascia comunque
        # un file utilizzabile fino all'ultimo campione.
        self.file = open(percorso, 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.file)
        self.writer.writerow(COLONNE)
        self.file.flush()
        return percorso

    # ------------------------------------------------------------- callback

    def _nuova_prova(self):
        """Chiude il file corrente e ne apre uno nuovo.

        Una prova e una missione: il nodo pero resta in piedi fra una missione e
        la successiva, e senza rotazione tutte finirebbero nello stesso file,
        rendendo impossibile confrontarle una a una. La rotazione scatta quando
        la missione lascia ATTESA, cioe all'avvio effettivo.
        """
        self.riepiloga()
        if getattr(self, 'file', None) and not self.file.closed:
            self.file.close()
        self.n_campioni = 0
        self.n_visibili = 0
        self.somma_dist = 0.0
        self.n_dist = 0
        self.tempo_per_fase = {}
        self.istante_cambio_fase = None
        self.percorso = self._apri_csv()
        self.get_logger().info('Nuova prova -> {}'.format(self.percorso))

    def on_stato(self, msg: String):
        # Il topic porta "PATTUGLIAMENTO:2": si registra la sola fase, il
        # numero di waypoint e ricavabile dalla posizione.
        fase = msg.data.split(':')[0]
        if (fase != 'ATTESA' and self.fase == 'ATTESA'
                and self.n_campioni > 0):
            self._nuova_prova()
        if fase != self.fase:
            adesso = self._ora_sim()
            if self.istante_cambio_fase is not None:
                self.tempo_per_fase[self.fase] = (
                    self.tempo_per_fase.get(self.fase, 0.0)
                    + (adesso - self.istante_cambio_fase))
            self.istante_cambio_fase = adesso
            self.fase = fase

    def on_posa(self, msg: PoseStamped):
        p = msg.pose.position
        self.ekf = (p.x, p.y, p.z)
        q = msg.pose.orientation
        self.pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        self.roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                               1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_velocita_drone(self, msg: TwistStamped):
        self.vel_drone = (msg.twist.linear.x, msg.twist.linear.y)

    def on_gimbal_roll(self, msg: Float64):
        self.gimbal_roll = msg.data

    def on_gimbal_pitch(self, msg: Float64):
        self.gimbal_pitch = msg.data

    def on_detection(self, msg: Point):
        self.det = (msg.x, msg.y, msg.z)

    def on_disturbata(self, msg: Point):
        self.det_disturbata = (msg.x, msg.y, msg.z)

    def on_rumore(self, msg):
        self.rumore_rf = msg.data

    def on_nis(self, msg):
        self.nis = msg.data
        self.n_det += 1

    def on_tracked(self, msg: Point):
        self.trk = (msg.x, msg.y, msg.z)
        self.n_trk += 1

    def on_tracked_vel(self, msg: Point):
        self.trk_vel = (msg.x, msg.y) if msg.z != 0.0 else None

    def on_jam(self, msg: Bool):
        self.jam = bool(msg.data)

    def on_denial(self, msg: Bool):
        self.gps_negato = bool(msg.data)

    def on_pose_info(self, msg):
        """Callback di gz-transport: gira su un thread proprio, non su quello di
        rclpy. Si limita a copiare due terne di float, operazione per cui il
        blocco globale dell'interprete basta: nessuna struttura viene letta a
        meta aggiornamento."""
        adesso = time.time()
        if adesso - self.ultima_posa_gz < self.periodo_posa_gz:
            return
        self.ultima_posa_gz = adesso
        for posa in msg.pose:
            if posa.name == self.nome_drone:
                self.gt_drone = (posa.position.x, posa.position.y, posa.position.z)
            elif posa.name == self.nome_bersaglio:
                self.gt_bersaglio = (posa.position.x, posa.position.y, posa.position.z)

    # --------------------------------------------------------- campionamento

    def _ora_sim(self):
        return self.get_clock().now().nanoseconds / 1e9

    def campiona(self):
        t_sim = self._ora_sim()
        t_wall = time.time() - self.t0_wall
        dt = 1.0 / self.frequenza

        det_valido = 1 if (self.det and self.det[2] != 0.0) else 0
        trk_valido = 1 if (self.trk and (self.trk[0] != 0.0 or self.trk[1] != 0.0)) else 0

        dist_xy_gt = dist_3d_gt = dist_xy_ekf = ''
        if self.gt_drone and self.gt_bersaglio:
            dx = self.gt_drone[0] - self.gt_bersaglio[0]
            dy = self.gt_drone[1] - self.gt_bersaglio[1]
            dz = self.gt_drone[2] - self.gt_bersaglio[2]
            dist_xy_gt = round(math.hypot(dx, dy), 3)
            dist_3d_gt = round(math.sqrt(dx * dx + dy * dy + dz * dz), 3)
            self.somma_dist += dist_xy_gt
            self.n_dist += 1
        if self.ekf and self.gt_bersaglio:
            # Il progetto assume che il riferimento locale di MAVROS coincida
            # con quello del mondo Gazebo. Questa colonna accanto a dist_xy_gt
            # e la verifica di quell'assunzione, non una misura ripetuta.
            dist_xy_ekf = round(math.hypot(self.ekf[0] - self.gt_bersaglio[0],
                                           self.ekf[1] - self.gt_bersaglio[1]), 3)

        def terna(v):
            return [round(c, 3) for c in v] if v else list(VUOTA)

        riga = [round(t_sim, 3), round(t_wall, 3), self.fase]
        riga += [det_valido] + terna(self.det) + [round(self.n_det / dt, 1)]
        d = self.det_disturbata
        riga += [1 if (d and d[2] != 0.0) else 0]
        riga += ([round(d[0], 4), round(d[1], 4)] if d else ['', ''])
        riga += [round(self.rumore_rf, 4) if self.rumore_rf is not None else '']
        riga += [round(self.nis, 3) if self.nis is not None else '']
        riga += [trk_valido] + terna(self.trk) + [round(self.n_trk / dt, 1)]
        riga += ([round(v, 4) for v in self.trk_vel] if self.trk_vel
                 else ['', ''])
        riga += terna(self.ekf)
        riga += [round(v, 4) if v is not None else ''
                 for v in (self.roll, self.pitch, self.yaw)]
        riga += ([round(v, 4) for v in self.vel_drone] if self.vel_drone
                 else ['', ''])
        riga += [round(v, 4) if v is not None else ''
                 for v in (self.gimbal_roll, self.gimbal_pitch)]
        riga += terna(self.gt_drone)
        riga += terna(self.gt_bersaglio)
        riga += [dist_xy_gt, dist_3d_gt, dist_xy_ekf]
        riga += [1 if self.jam else 0, 1 if self.gps_negato else 0]

        # Una riga piu corta dell'intestazione non darebbe errore: sposterebbe
        # i valori di una casella, e il CSV mentirebbe senza dirlo.
        if len(riga) != len(COLONNE):
            raise RuntimeError(
                'riga da %d valori contro %d colonne dichiarate'
                % (len(riga), len(COLONNE)))

        self.writer.writerow(riga)
        self.file.flush()

        self.n_campioni += 1
        self.n_visibili += det_valido
        self.n_det = 0
        self.n_trk = 0

    # -------------------------------------------------------------- chiusura

    def riepiloga(self):
        if self.istante_cambio_fase is not None:
            self.tempo_per_fase[self.fase] = (
                self.tempo_per_fase.get(self.fase, 0.0)
                + (self._ora_sim() - self.istante_cambio_fase))

        if self.n_campioni == 0:
            self.get_logger().warn('Nessun campione registrato.')
            return

        visibile = 100.0 * self.n_visibili / self.n_campioni
        media = (self.somma_dist / self.n_dist) if self.n_dist else float('nan')
        fasi = ', '.join('{}={:.1f}s'.format(k, v)
                         for k, v in sorted(self.tempo_per_fase.items()))
        self.get_logger().info(
            '--- Riepilogo prova ---\n'
            '  file:             {}\n'
            '  campioni:         {}\n'
            '  bersaglio visto:  {:.1f}% dei campioni\n'
            '  distanza media:   {:.2f} m (verita a terra, orizzontale)\n'
            '  tempo per fase:   {}'.format(
                self.percorso, self.n_campioni, visibile, media, fasi))

    def destroy_node(self):
        try:
            self.riepiloga()
        finally:
            if getattr(self, 'file', None) and not self.file.closed:
                self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MetricsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
