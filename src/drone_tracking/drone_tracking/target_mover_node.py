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
from drone_tracking.parametri import parametro  # type: ignore

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
        self.pos_x = 150.0
        self.pos_y = 150.0
        self.drone_x = 0.0
        self.drone_y = 0.0
        
        # Ritardo prima della fuga, in SECONDI. Contava messaggi di
        # /mission/stato, che arrivano a 2 Hz e non a 10: 50 conteggi facevano
        # 25 secondi invece dei 5 dichiarati. Ora si misura il tempo reale, così
        # il valore non dipende dalla frequenza del topic.
        self.istante_aggancio  = None
        self.ritardo_evasione_s = parametro(self, 'ritardo_evasione_s', 10.0)

        # I parametri di moto sono espressi in unità AL SECONDO e integrati sul
        # dt reale. Prima erano incrementi per tick, con il timer dichiarato a
        # 10 Hz: ma `gz service` impiega ~360 ms a rispondere e, chiamato in modo
        # bloccante, teneva il timer a ~2.8 Hz. I valori qui sotto riproducono la
        # velocità effettivamente osservata con quel timer strozzato.
        self.centro_x = 150.0
        self.centro_y = 150.0
        # Il centro si sposta dopo ogni evasione: quello iniziale va conservato
        # per poter riportare il bersaglio al punto di partenza.
        self.centro_iniziale = (self.centro_x, self.centro_y)
        # Si assume ATTESA fino a prova contraria: serve a riconoscere il
        # passaggio ATTESA -> missione avviata, non lo stato in se.
        self.missione_in_attesa = True
        # Raggio dell'orbita di pattuglia. Quaranta metri con 0.25 rad/s fanno
        # 10 m/s tangenziali, cioe 36 km/h: la velocita a cui un veicolo
        # pattuglia davvero. Lo scenario precedente aveva raggio 3 m e 1.05 m/s
        # tangenziali, cioe passo d'uomo su un cerchio piu piccolo del veicolo
        # che avrebbe dovuto rappresentare.
        self.raggio   = parametro(self, 'raggio_orbita', 40.0, 'raggio')
        self.velocita_angolare = parametro(
            self, 'velocita_angolare', 0.25)   # rad/s

        # Parametri evasione. Il limite non è la velocità massima del drone
        # ma l'errore a regime del controllo proporzionale, che vale circa
        # `velocita_bersaglio / kp`: più il bersaglio è veloce, più il drone lo
        # insegue da lontano, finché non esce dall'inquadratura.
        #
        # La fuga parte da ferma e accelera: prima la velocità veniva applicata
        # intera al primo istante, uno scalino sia in modulo sia in direzione
        # rispetto al moto tangenziale dell'orbita. Un veicolo che scappa
        # accelera, e la rampa dà al drone qualche secondo per reagire prima che
        # il bersaglio sia a piena velocità.
        # 15 m/s sono 54 km/h: la fuga di un veicolo su strada sterrata, non
        # piu quella di un pedone. L'accelerazione di 3 m/s^2 porta a regime in
        # cinque secondi, che e il comportamento di un mezzo leggero.
        self.vel_evasione = parametro(
            self, 'vel_evasione', 15.0)    # m/s a regime, ~54 km/h
        self.accel_evasione = parametro(
            self, 'accel_evasione', 3.0)   # m/s^2, 5 s per il regime
        self.dir_evasione_x    = 0.0
        self.dir_evasione_y    = 0.0
        self.tempo_evasione    = 0.0
        # A 8.3 m/s venti secondi porterebbero il bersaglio a 160 m, fuori da
        # qualunque possibilita di recupero. Dieci bastano a mettere alla prova
        # l'inseguimento senza trasformarlo in una fuga senza ritorno.
        # Venti secondi a 15 m/s portano il bersaglio a circa 260 m dal punto
        # di partenza: molto piu del semicampo inquadrato, quindi la fuga mette
        # davvero alla prova l'inseguimento invece di svolgersi tutta dentro
        # una sola inquadratura.
        self.durata_evasione_s = parametro(
            self, 'durata_evasione_s', 20.0)   # s  -> ~260 m di fuga

        self.ultimo_istante = None
        self.proc_pendente  = None

        self.nome_modello  = 'bersaglio'
        # Meta dell'altezza del veicolo: lo appoggia a terra invece di
        # interrarlo o farlo levitare.
        self.quota_modello = 0.8
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

    def _riparti_da_capo(self):
        """Riporta il bersaglio al punto di partenza dell'orbita.

        Il bersaglio si muove da quando il nodo e nato, mentre la missione parte
        quando lo decide l'operatore: due prove della stessa configurazione
        trovavano quindi il bersaglio in punti diversi dell'orbita, e quel solo
        scarto le rendeva inconfrontabili campione per campione.

        Il riporto avviene mentre la missione e ancora in ATTESA e viene
        ripetuto fino all'avvio, cosi il bersaglio e fermo al punto di partenza
        quando la registrazione comincia. Farlo all'istante dell'avvio, come
        nella prima versione, metteva un salto di parecchi metri nel primo
        campione della prova: la velocita di picco del bersaglio risultava di
        15 m/s con il parametro a 1.2.
        """
        self.fase = FaseBersaglio.PATTUGLIO
        self.t = 0.0
        self.centro_x, self.centro_y = self.centro_iniziale
        self.pos_x = self.centro_x + self.raggio
        self.pos_y = self.centro_y
        self.tempo_evasione = 0.0
        self.istante_aggancio = None

    def on_stato_missione(self, msg: String):
        in_attesa = FaseMissione.ATTESA.value in msg.data
        if in_attesa:
            # Finche la missione non parte il bersaglio resta al punto di
            # partenza: condizioni iniziali identiche a ogni prova.
            self._riparti_da_capo()
        elif self.missione_in_attesa:
            self.get_logger().info(
                'Missione avviata — il bersaglio parte da '
                '({:.1f}, {:.1f})'.format(self.pos_x, self.pos_y))
        self.missione_in_attesa = in_attesa

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

        # Missione non ancora avviata: il bersaglio sta dove e, ma la posa va
        # comunque comandata, altrimenti la fisica di Gazebo se ne impossessa.
        if self.missione_in_attesa:
            self._invia_posa(self.pos_x, self.pos_y, self.quota_modello)
            return

        if self.fase == FaseBersaglio.PATTUGLIO:
            self.t += self.velocita_angolare * dt
            self.pos_x = self.centro_x + self.raggio * math.cos(self.t)
            self.pos_y = self.centro_y + self.raggio * math.sin(self.t)

        elif self.fase == FaseBersaglio.EVASIONE:
            self.tempo_evasione += dt
            # Rampa di accelerazione, satura alla velocità di regime.
            v = min(self.vel_evasione, self.accel_evasione * self.tempo_evasione)
            self.pos_x += self.dir_evasione_x * v * dt
            self.pos_y += self.dir_evasione_y * v * dt

            if self.tempo_evasione >= self.durata_evasione_s:
                self.fase = FaseBersaglio.PATTUGLIO
                # L'orbita riprende dal punto in cui la fuga si e fermata.
                # Prima il centro veniva messo sulla posizione corrente con
                # t = 0, e la posizione successiva valeva centro + raggio:
                # un salto istantaneo di 3 metri, cioe l'intero raggio
                # dell'orbita. Il tracker lo vedeva come uno spostamento
                # impossibile del bersaglio e poteva perdere l'aggancio per un
                # artefatto del simulatore, non per un limite del controllo.
                # Mettendo il centro dietro la direzione di fuga e la fase
                # dell'orbita pari a quella direzione, la posizione resta
                # invariata e il moto prosegue senza strappi.
                self.t = math.atan2(self.dir_evasione_y, self.dir_evasione_x)
                self.centro_x = self.pos_x - self.raggio * math.cos(self.t)
                self.centro_y = self.pos_y - self.raggio * math.sin(self.t)
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