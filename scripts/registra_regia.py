#!/usr/bin/env python3
"""
Registra la missione come la seguirebbe una regia, dall'allerta al rientro:
telecamera di bordo a sinistra, mappa dall'alto a destra, console dei nodi
ROS in basso. Serve a capire le dinamiche del volo e a mostrarle, decollo,
aggancio e rientro compresi, senza dover leggere sei terminali insieme.

    registra_regia.py [--durata S] [--rientro S] [--calma X]

    registra_regia.py                    # fino a 300 s simulati
    registra_regia.py --rientro 150      # 150 s dopo l'avvio della missione
                                         # comanda il rientro e chiude
                                         # quando il drone e atterrato
    registra_regia.py --calma 1          # niente tratti accelerati

Va lanciato PRIMA che la stazione faccia decollare il drone, cosi il video
comincia dall'allerta. Il file finisce in /ws/snapshots nel container, in
snapshots/ del repository altrove. Ctrl-C chiude e salva quello che c'e.

Il tempo e quello SIMULATO: un secondo simulato dura un secondo di video,
qualunque sia la velocita della macchina. I tratti senza novita (attesa a
terra, salita, trasferimento, volo di rientro) scorrono piu veloci, con il
fattore scritto sul fotogramma: la giuria deve sapere che il video e
accelerato, non scoprirlo. L'aggancio e ogni cambio di stato restano a
velocita normale, la ricerca va al doppio.

Il rientro passa da /sicurezza/rientro, cioe dalla stessa strada di un limite
superato: missione e controllo smettono di spingere e il drone torna in RTL.
"""
import argparse
import bisect
from collections import deque
import math
import os
import sys
import tempfile
import threading
import time
import unicodedata

import cv2
import numpy as np

LARGHEZZA, ALTEZZA = 1280, 720
BANDA = 56                        # fascia di stato in alto
PANNELLO_W, PANNELLO_H = 640, 480
CONSOLE_Y = BANDA + PANNELLO_H    # la console occupa il resto
RIGHE_CONSOLE = 8
CAMPIONI_AL_SECONDO = 8           # fotogrammi per secondo simulato
FPS_USCITA = 16

RAGGIO_OPERATIVO_M = 440.0        # mission_node, raggio_operativo_m
CONFINE_M = 500.0                 # sicurezza_node, raggio_max

# Intorno a un evento il video torna a velocita normale: prima per vedere
# arrivare il cambiamento, dopo per vederne l'effetto.
PRIMA_EVENTO_S, DOPO_EVENTO_S = 2.0, 5.0

NODI_PROGETTO = {
    'stazione_node', 'mission_node', 'sicurezza_node', 'controller_node',
    'tracker_node', 'detector_node', 'target_mover_node', 'metrics_node',
    'jammer_node', 'gimbal_node', 'gnss_denial_node',
}
INFO, WARN, ERROR = 20, 30, 40

# Colori in BGR
NERO = (20, 20, 20)
GRIGIO_SCURO = (48, 48, 48)
GRIGIO = (140, 140, 140)
BIANCO = (235, 235, 235)
ROSSO = (60, 60, 225)
ARANCIO = (0, 150, 255)
GIALLO = (0, 215, 255)
VERDE = (90, 210, 90)
AZZURRO = (255, 210, 60)

FONT = cv2.FONT_HERSHEY_SIMPLEX


# ------------------------------------------------------------ parti pure
def leggibile(testo):
    """Il testo ridotto ad ASCII: OpenCV non sa disegnare accenti e trattini lunghi."""
    testo = testo.replace('—', '-').replace('–', '-').replace('°', ' gradi')
    return unicodedata.normalize('NFKD', testo).encode('ascii', 'ignore').decode()


def riga_console(nome, livello, testo):
    """Il testo da mostrare per un messaggio di /rosout, o None se e rumore.

    Dei nodi del progetto passa tutto da INFO in su. Di MAVROS solo due cose:
    i messaggi dell'autopilota (armamento, EKF, modi rifiutati) e la perdita
    o il ritorno del collegamento. Il resto di MAVROS, come la sincronizzazione
    dei parametri, riempirebbe la console senza dire nulla del volo.
    """
    if nome in NODI_PROGETTO:
        return testo if livello >= INFO else None
    if nome == 'mavros.sys' and testo.startswith('FCU:'):
        return testo
    if nome == 'mavros' and testo.startswith('CON:'):
        return testo
    return None


def aggiungi_riga(righe, t, livello, nome, testo, massimo=RIGHE_CONSOLE):
    """Aggiunge una riga alla console, raggruppando i messaggi dello stesso tipo.

    In ricerca mission_node scrive un nuovo punto della spirale ogni mezzo
    secondo e la stazione un'allerta ogni due: senza raggruppare, otto righe
    di console erano sempre e solo spirale. Due messaggi sono dello stesso
    tipo se vengono dallo stesso nodo e cominciano con le stesse tre parole;
    la riga si aggiorna, scende in fondo e conta le ripetizioni.
    """
    chiave = (nome, tuple(testo.split()[:3]))
    volte = 1
    for k, riga in enumerate(righe):
        if (riga[2], tuple(riga[3].split()[:3])) == chiave:
            volte = riga[4] + 1
            del righe[k]
            break
    righe.append((t, livello, nome, testo, volte))
    del righe[:-massimo]
    return righe


def nome_breve(nome):
    if nome == 'mavros.sys':
        return 'autopilota'
    return nome[:-5] if nome.endswith('_node') else nome


def velocita_riproduzione(tempi, fasi, eventi, calma):
    """Fattore di accelerazione di ogni fotogramma.

    Normale in aggancio e attorno a ogni evento, doppia in ricerca, `calma`
    altrove. La ricerca resta piu lenta del resto perche e li che si vede il
    drone inseguire le allerte, ma a velocita normale una ricerca di due
    minuti teneva in piedi da sola mezzo video.
    """
    eventi = sorted(eventi)
    risultato = []
    for t, fase in zip(tempi, fasi):
        if fase.startswith('AGGANCIO'):
            risultato.append(1.0)
            continue
        # Primo evento non precedente a t - DOPO: e vicino se cade entro t + PRIMA.
        k = bisect.bisect_left(eventi, t - DOPO_EVENTO_S)
        vicino = k < len(eventi) and eventi[k] <= t + PRIMA_EVENTO_S
        if vicino:
            risultato.append(1.0)
        elif fase.startswith('RICERCA'):
            risultato.append(min(2.0, calma))
        else:
            risultato.append(calma)
    return risultato


def piano_uscita(tempi, velocita, fps):
    """Indici dei fotogrammi da scrivere, con ripetizioni, a fps costanti.

    Ogni fotogramma copre il tempo simulato fino al successivo, diviso per la
    sua velocita. Un buco lungo (simulazione ferma, Ctrl-Z) non diventa una
    pausa nel video: oltre un secondo si tronca.
    """
    if not tempi:
        return []
    passi = [b - a for a, b in zip(tempi, tempi[1:])]
    tipico = sorted(passi)[len(passi) // 2] if passi else 1.0 / CAMPIONI_AL_SECONDO
    indici, accumulato, scritti = [], 0.0, 0
    for i, v in enumerate(velocita):
        durata = passi[i] if i < len(passi) else tipico
        accumulato += min(max(durata, 0.0), 1.0) / v
        while scritti < round(accumulato * fps):
            indici.append(i)
            scritti += 1
    return indici


def imbardata(q):
    """Angolo di rotta (rad, da est in senso antiorario) da un quaternione ENU."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Mappa:
    """Vista dall'alto centrata sulla stazione, con il nord in alto."""

    def __init__(self, larghezza=PANNELLO_W, altezza=PANNELLO_H, raggio_m=560.0):
        self.w, self.h = larghezza, altezza
        self.cx, self.cy = larghezza // 2, altezza // 2
        self.inquadra(raggio_m)

    def inquadra(self, raggio_m):
        """Metri dal centro al bordo corto del pannello."""
        self.raggio_m = raggio_m
        self.scala = (min(self.w, self.h) / 2 - 12) / raggio_m

    @staticmethod
    def raggio_utile(punti, minimo=220.0, massimo=560.0):
        """Raggio di vista che contiene tutti i punti, con margine.

        Con la vista fissa sul confine tutta l'azione dei primi minuti, entro
        250 m, occupava un quinto del pannello. Si allarga solo quando drone,
        bersaglio o allerte si allontanano.
        """
        lontano = max((math.hypot(x, y) for x, y in punti), default=0.0)
        return min(massimo, max(minimo, lontano * 1.25 + 40.0))

    def px(self, x, y):
        return (int(round(self.cx + x * self.scala)),
                int(round(self.cy - y * self.scala)))

    def _cerchio_tratteggiato(self, img, raggio_m, colore):
        r = int(raggio_m * self.scala)
        for a in range(0, 360, 10):
            cv2.ellipse(img, (self.cx, self.cy), (r, r), 0, a, a + 5, colore, 1, cv2.LINE_AA)

    def disegna(self, s):
        img = np.full((self.h, self.w, 3), 28, np.uint8)
        for m in range(-800, 801, 100):
            x0, _ = self.px(m, 0)
            _, y0 = self.px(0, m)
            cv2.line(img, (x0, 0), (x0, self.h), GRIGIO_SCURO, 1)
            cv2.line(img, (0, y0), (self.w, y0), GRIGIO_SCURO, 1)
        self._cerchio_tratteggiato(img, RAGGIO_OPERATIVO_M, GRIGIO)
        cv2.circle(img, (self.cx, self.cy), int(CONFINE_M * self.scala), ROSSO, 1, cv2.LINE_AA)
        etichetta = self.px(CONFINE_M * 0.72, CONFINE_M * 0.72 + 8)
        if 0 <= etichetta[0] < self.w - 110 and 30 <= etichetta[1] < self.h:
            cv2.putText(img, 'confine 500 m', etichetta, FONT, 0.4, ROSSO, 1, cv2.LINE_AA)

        # Allerte: le vecchie sbiadiscono, l'ultima porta il cerchio d'errore.
        for k, (t, x, y, sigma) in enumerate(s['allerte']):
            vecchia = s['adesso'] - t
            if vecchia > 30:
                continue
            peso = max(0.25, 1.0 - vecchia / 30)
            colore = tuple(int(c * peso) for c in ARANCIO)
            cv2.circle(img, self.px(x, y), 3, colore, -1, cv2.LINE_AA)
            if k == len(s['allerte']) - 1:
                cv2.circle(img, self.px(x, y), max(3, int(sigma * self.scala)),
                           ARANCIO, 1, cv2.LINE_AA)

        for scia, colore in ((s['scia_bersaglio'], ROSSO), (s['scia_drone'], AZZURRO)):
            punti = [self.px(x, y) for x, y in scia]
            if len(punti) > 1:
                cv2.polylines(img, [np.array(punti, np.int32)], False,
                              tuple(int(c * 0.55) for c in colore), 1, cv2.LINE_AA)

        # Stazione a terra
        sx, sy = self.px(0, 0)
        cv2.rectangle(img, (sx - 5, sy - 5), (sx + 5, sy + 5), BIANCO, -1)
        cv2.putText(img, 'stazione', (sx + 9, sy + 16), FONT, 0.4, BIANCO, 1, cv2.LINE_AA)

        if s['bersaglio'] is not None:
            cv2.circle(img, self.px(*s['bersaglio']), 6, ROSSO, -1, cv2.LINE_AA)
        if s['drone'] is not None:
            dx, dy = self.px(*s['drone'])
            if s['bersaglio'] is not None and s['fase'].startswith('AGGANCIO'):
                cv2.line(img, (dx, dy), self.px(*s['bersaglio']), VERDE, 1, cv2.LINE_AA)
            a = s['rotta']
            punta = (dx + 11 * math.cos(a), dy - 11 * math.sin(a))
            sin1 = (dx + 7 * math.cos(a + 2.5), dy - 7 * math.sin(a + 2.5))
            sin2 = (dx + 7 * math.cos(a - 2.5), dy - 7 * math.sin(a - 2.5))
            cv2.fillPoly(img, [np.array([punta, sin1, sin2], np.int32)], AZZURRO, cv2.LINE_AA)

        # Legenda e scala
        voci = (('drone', AZZURRO), ('bersaglio (vero)', ROSSO),
                ('allerte della stazione', ARANCIO), ('limite operativo 440 m', GRIGIO))
        for k, (testo, colore) in enumerate(voci):
            y = self.h - 14 - 18 * (len(voci) - 1 - k)
            cv2.circle(img, (16, y - 4), 4, colore, -1, cv2.LINE_AA)
            cv2.putText(img, testo, (28, y), FONT, 0.42, BIANCO, 1, cv2.LINE_AA)
        lun = int(100 * self.scala)
        x1, y1 = self.w - 20 - lun, self.h - 18
        cv2.line(img, (x1, y1), (x1 + lun, y1), BIANCO, 2)
        cv2.putText(img, '100 m', (x1 + lun // 2 - 20, y1 - 8), FONT, 0.42, BIANCO, 1, cv2.LINE_AA)
        cv2.putText(img, 'N', (self.w - 22, 22), FONT, 0.55, BIANCO, 1, cv2.LINE_AA)
        cv2.putText(img, "mappa dall'alto", (10, 22), FONT, 0.5, BIANCO, 1, cv2.LINE_AA)
        return img


# -------------------------------------------------------------- nodo ROS
def crea_registratore(cartella_frame, rientro_s):
    import rclpy  # noqa: F401  (gia inizializzato da main)
    from cv_bridge import CvBridge
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from mavros_msgs.msg import State
    from rcl_interfaces.msg import Log
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import Bool, Float64, String

    try:
        from gz.transport13 import Node as GzNode
        from gz.msgs10.pose_v_pb2 import Pose_V
    except ImportError:
        GzNode = None

    class Registratore(Node):
        def __init__(self):
            super().__init__('registra_regia', parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)])
            sensore = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST, depth=1)
            self.bridge = CvBridge()
            self.mappa = Mappa(raggio_m=220.0)
            self.cartella = cartella_frame
            self.rientro_s = rientro_s

            self.img_grezza = None
            self.img_rilevatore = None       # (istante simulato, messaggio)
            self.fase = 'ATTESA'
            self.staz = None
            self.modo = '?'
            self.armato = False
            self.quota = None
            self.sicurezza = None
            self.drone = None
            self.rotta = math.pi / 2
            self.allerte = deque(maxlen=40)
            self.scia_drone = deque(maxlen=600)
            self.scia_bersaglio = deque(maxlen=600)
            self.console = []

            self.t0 = None
            self.ultimo_campione = None
            self.inizio_missione = None
            self.rientro_inviato = None
            self.atterrato = None
            self.fotogrammi = []             # (t relativo, fase, percorso)
            self.eventi = []                 # istanti relativi
            self.ultimo_arrivo = time.monotonic()

            self.lucchetto = threading.Lock()
            self.verita = None
            self.gz = None
            if GzNode is not None:
                self.gz = GzNode()
                self.gz.subscribe(Pose_V, '/world/iris_runway/pose/info', self.su_pose_gz)

            self.create_subscription(Image, '/drone/camera/image_raw', self.su_grezza, sensore)
            self.create_subscription(Image, '/target/debug_image', self.su_rilevatore, sensore)
            self.create_subscription(String, '/mission/stato', self.su_fase, 10)
            self.create_subscription(String, '/stazione/stato', self.su_staz, 10)
            self.create_subscription(String, '/sicurezza/stato', self.su_sicurezza, 10)
            self.create_subscription(PoseWithCovarianceStamped, '/stazione/allerta',
                                     self.su_allerta, 10)
            self.create_subscription(State, '/mavros/state', self.su_stato_volo, sensore)
            self.create_subscription(Float64, '/mavros/global_position/rel_alt',
                                     self.su_quota, sensore)
            self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                     self.su_posa, sensore)
            self.create_subscription(Log, '/rosout', self.su_log, 100)
            self.pub_rientro = self.create_publisher(Bool, '/sicurezza/rientro', 10)
            self.Bool = Bool

        # ------------------------------------------------------- ingressi
        def adesso(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def relativo(self):
            return self.adesso() - self.t0 if self.t0 is not None else 0.0

        def evento(self):
            self.eventi.append(self.relativo())

        def su_pose_gz(self, msg):
            for posa in msg.pose:
                if posa.name == 'bersaglio':
                    with self.lucchetto:
                        self.verita = (posa.position.x, posa.position.y)
                    return

        def su_grezza(self, msg):
            self.img_grezza = msg

        def su_rilevatore(self, msg):
            self.img_rilevatore = (self.adesso(), msg)

        def su_fase(self, msg):
            if msg.data != self.fase:
                self.evento()
                if self.inizio_missione is None and not msg.data.startswith('ATTESA'):
                    self.inizio_missione = self.relativo()
            self.fase = msg.data

        def su_staz(self, msg):
            if msg.data != self.staz:
                self.evento()
            self.staz = msg.data

        def su_sicurezza(self, msg):
            if self.sicurezza is not None and msg.data != self.sicurezza:
                self.evento()
            self.sicurezza = msg.data

        def su_allerta(self, msg):
            p = msg.pose.pose.position
            istante = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.allerte.append((istante, p.x, p.y, msg.pose.covariance[0] ** 0.5))

        def su_stato_volo(self, msg):
            if msg.mode != self.modo or msg.armed != self.armato:
                self.evento()
            if self.armato and not msg.armed and self.rientro_inviato is not None:
                self.atterrato = self.relativo()
            self.modo, self.armato = msg.mode, msg.armed

        def su_quota(self, msg):
            self.quota = msg.data

        def su_posa(self, msg):
            p = msg.pose.position
            self.drone = (p.x, p.y)
            self.rotta = imbardata(msg.pose.orientation)

        def su_log(self, msg):
            testo = riga_console(msg.name, msg.level, msg.msg)
            if testo is None:
                return
            aggiungi_riga(self.console, self.relativo(), msg.level,
                          nome_breve(msg.name), leggibile(testo))
            # Solo gli errori fanno rallentare: gli avvisi (bersaglio perso,
            # disturbo attivo) arrivano ogni pochi secondi e terrebbero il
            # video sempre a velocita normale. I cambi di stato sono gia eventi.
            if msg.level >= ERROR:
                self.evento()

        # ------------------------------------------------- fotogramma
        def passo(self):
            """Da chiamare spesso: campiona, e se serve comanda il rientro."""
            adesso = self.adesso()
            if adesso <= 0.0:
                return              # /clock non ancora arrivato
            if self.t0 is None:
                self.t0 = adesso
            if (self.ultimo_campione is not None
                    and adesso - self.ultimo_campione < 1.0 / CAMPIONI_AL_SECONDO):
                return
            self.ultimo_campione = adesso
            with self.lucchetto:
                verita = self.verita
            if verita is not None:
                if not self.scia_bersaglio or math.dist(self.scia_bersaglio[-1], verita) > 2:
                    self.scia_bersaglio.append(verita)
            if self.drone is not None and self.armato:
                if not self.scia_drone or math.dist(self.scia_drone[-1], self.drone) > 2:
                    self.scia_drone.append(self.drone)

            if (self.rientro_s > 0 and self.rientro_inviato is None
                    and self.inizio_missione is not None
                    and self.relativo() - self.inizio_missione >= self.rientro_s):
                self.rientro_inviato = self.relativo()
                print(f'  {self.relativo():.0f} s: comando il rientro', flush=True)
            if self.rientro_inviato is not None and self.atterrato is None:
                # Ripetuto finche non atterra: un messaggio perso non lascia
                # il drone a inseguire. Il nodo di sicurezza ignora i doppioni.
                self.pub_rientro.publish(self.Bool(data=True))

            img = self.componi(verita)
            percorso = os.path.join(self.cartella, f'{len(self.fotogrammi):06d}.jpg')
            cv2.imwrite(percorso, img, [cv2.IMWRITE_JPEG_QUALITY, 85])
            self.fotogrammi.append((self.relativo(), self.fase, percorso))
            self.ultimo_arrivo = time.monotonic()
            if len(self.fotogrammi) % (CAMPIONI_AL_SECONDO * 15) == 0:
                print(f'  {self.relativo():6.0f} s simulati  stazione={self.staz}  '
                      f'missione={self.fase}  modo={self.modo}', flush=True)

        def immagine_camera(self):
            if self.img_rilevatore is not None and self.adesso() - self.img_rilevatore[0] < 1.5:
                return self.img_rilevatore[1], 'vista del rilevatore'
            if self.img_grezza is not None:
                return self.img_grezza, 'telecamera di bordo'
            return None, 'telecamera: in attesa'

        def componi(self, verita):
            tela = np.full((ALTEZZA, LARGHEZZA, 3), NERO[0], np.uint8)
            t = self.relativo()

            # Telecamera
            msg, etichetta = self.immagine_camera()
            if msg is not None:
                cam = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                if cam.shape[:2] != (PANNELLO_H, PANNELLO_W):
                    cam = cv2.resize(cam, (PANNELLO_W, PANNELLO_H))
                tela[BANDA:BANDA + PANNELLO_H, :PANNELLO_W] = cam
            cv2.rectangle(tela, (0, BANDA), (230, BANDA + 26), NERO, -1)
            cv2.putText(tela, etichetta, (10, BANDA + 19), FONT, 0.5, BIANCO, 1, cv2.LINE_AA)

            # Mappa: la vista segue il punto piu lontano, smorzata perche
            # lo zoom non pulsi a ogni allerta.
            punti = [p for p in (self.drone, verita) if p is not None]
            punti += [(x, y) for ist, x, y, _ in self.allerte if self.adesso() - ist < 30]
            voluto = Mappa.raggio_utile(punti)
            self.mappa.inquadra(self.mappa.raggio_m + 0.08 * (voluto - self.mappa.raggio_m))
            stato = {
                'adesso': self.adesso(), 'allerte': list(self.allerte),
                'scia_drone': self.scia_drone, 'scia_bersaglio': self.scia_bersaglio,
                'bersaglio': verita, 'drone': self.drone, 'rotta': self.rotta,
                'fase': self.fase,
            }
            tela[BANDA:BANDA + PANNELLO_H, PANNELLO_W:] = self.mappa.disegna(stato)
            cv2.line(tela, (PANNELLO_W, BANDA), (PANNELLO_W, CONSOLE_Y), GRIGIO, 1)

            # Fascia di stato
            quota = f'{self.quota:5.1f} m' if self.quota is not None else '? m'
            dist = f'{math.hypot(*self.drone):4.0f} m' if self.drone else '? m'
            riga1 = (f't {t:6.1f} s   stazione {self.staz or "-"}   missione {self.fase}   '
                     f'modo {self.modo}{" armato" if self.armato else ""}   '
                     f'quota {quota}   dalla stazione {dist}')
            cv2.putText(tela, riga1, (12, 23), FONT, 0.55, BIANCO, 1, cv2.LINE_AA)
            sic = self.sicurezza or '-'
            colore = VERDE if sic == 'entro i limiti' else ROSSO
            cv2.putText(tela, f'sicurezza: {leggibile(sic)}', (12, 47), FONT, 0.5, colore, 1,
                        cv2.LINE_AA)
            if self.allerte:
                ist, x, y, sigma = self.allerte[-1]
                testo = (f'ultima allerta ({x:.0f}, {y:.0f}) +/-{sigma:.0f} m, '
                         f'{self.adesso() - ist:.1f} s fa')
                cv2.putText(tela, testo, (560, 47), FONT, 0.5, ARANCIO, 1, cv2.LINE_AA)

            # Console
            cv2.line(tela, (0, CONSOLE_Y), (LARGHEZZA, CONSOLE_Y), GRIGIO, 1)
            cv2.putText(tela, 'console dei nodi ROS', (12, CONSOLE_Y + 18), FONT, 0.45,
                        GRIGIO, 1, cv2.LINE_AA)
            for k, (tr, livello, nome, testo, volte) in enumerate(self.console):
                colore = ROSSO if livello >= ERROR else GIALLO if livello >= WARN else BIANCO
                ripetuto = f'  (x{volte})' if volte > 1 else ''
                riga = f'[{tr:6.1f}] {nome:<11} {testo}{ripetuto}'
                cv2.putText(tela, riga[:150], (12, CONSOLE_Y + 38 + 19 * k), FONT, 0.43,
                            colore, 1, cv2.LINE_AA)
            return tela

    return Registratore()


# ------------------------------------------------------------- scrittura
def scrivi_video(fotogrammi, eventi, calma, destinazione):
    tempi = [f[0] for f in fotogrammi]
    fasi = [f[1] for f in fotogrammi]
    velocita = velocita_riproduzione(tempi, fasi, eventi, calma)
    indici = piano_uscita(tempi, velocita, FPS_USCITA)
    scrittore = cv2.VideoWriter(destinazione, cv2.VideoWriter_fourcc(*'mp4v'),
                                FPS_USCITA, (LARGHEZZA, ALTEZZA))
    if not scrittore.isOpened():
        raise RuntimeError(f'OpenCV non riesce a scrivere {destinazione}')
    precedente, img = None, None
    for i in indici:
        if i != precedente:
            img = cv2.imread(fotogrammi[i][2])
            if velocita[i] > 1.0:
                testo = f'x{velocita[i]:.0f} >>'
                # In alto a destra della telecamera: la mappa li ha la sua rosa.
                cv2.rectangle(img, (PANNELLO_W - 108, BANDA + 4), (PANNELLO_W - 6, BANDA + 34),
                              NERO, -1)
                cv2.putText(img, testo, (PANNELLO_W - 98, BANDA + 27), FONT, 0.7, GIALLO, 2,
                            cv2.LINE_AA)
            precedente = i
        scrittore.write(img)
    scrittore.release()
    accelerati = sum(1 for v in velocita if v > 1.0) / max(1, len(velocita))
    return len(indici) / FPS_USCITA, accelerati


def cartella_predefinita():
    if os.path.isdir('/ws/snapshots'):
        return '/ws/snapshots'
    radice = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(radice, 'snapshots')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--durata', type=float, default=300.0,
                        help='secondi simulati massimi (default 300)')
    parser.add_argument('--rientro', type=float, default=0.0,
                        help="secondi dopo l'avvio della missione in cui comandare "
                             'il rientro; 0 = mai (default)')
    parser.add_argument('--calma', type=float, default=4.0,
                        help='accelerazione dei tratti senza novita (default 4)')
    args = parser.parse_args()

    import rclpy
    cartella = os.environ.get('CARTELLA_VIDEO', cartella_predefinita())
    os.makedirs(cartella, exist_ok=True)
    destinazione = os.path.join(cartella, time.strftime('regia_%Y%m%d_%H%M%S.mp4'))

    with tempfile.TemporaryDirectory(prefix='registra_regia_') as tmp:
        rclpy.init()
        nodo = crea_registratore(tmp, args.rientro)
        print(f'Registro la regia per al massimo {args.durata:.0f} s simulati'
              + (f", rientro {args.rientro:.0f} s dopo l'avvio della missione"
                 if args.rientro > 0 else '') + ' (Ctrl-C per chiudere prima)...',
              flush=True)
        try:
            while rclpy.ok():
                rclpy.spin_once(nodo, timeout_sec=0.02)
                nodo.passo()
                if nodo.relativo() >= args.durata:
                    break
                if nodo.atterrato is not None and nodo.relativo() - nodo.atterrato > 4.0:
                    print('  atterrato e disarmato: chiudo.', flush=True)
                    break
                if nodo.fotogrammi and time.monotonic() - nodo.ultimo_arrivo > 60:
                    print('  simulazione ferma da 60 s: chiudo.', flush=True)
                    break
        except KeyboardInterrupt:
            print('\n  interrotto: salvo quello che ho.', flush=True)
        finally:
            fotogrammi, eventi = nodo.fotogrammi, nodo.eventi
            nodo.destroy_node()
            rclpy.try_shutdown()

        if len(fotogrammi) < 2:
            print('Nessun video: meno di due fotogrammi. /clock arriva?', file=sys.stderr)
            sys.exit(1)
        durata_video, accelerati = scrivi_video(fotogrammi, eventi, args.calma, destinazione)
        print(f'Fatto: {destinazione}')
        print(f'  {fotogrammi[-1][0]:.0f} s simulati in {durata_video:.0f} s di video, '
              f'{100 * accelerati:.0f}% del tempo accelerato x{args.calma:.0f}')


if __name__ == '__main__':
    main()
