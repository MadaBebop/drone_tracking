#!/usr/bin/env python3
"""
Registra cio che vede il rilevatore in un video MP4, con fase della missione,
quota e tempo simulato sovrimpressi. Serve come piano B per una demo dal vivo:
se la rete o il portatile tradiscono, si proietta il video.

    registra_video.py [secondi_simulati] [velocita] [topic]

    registra_video.py              # 120 s simulati da /target/debug_image
    registra_video.py 180 2        # 180 s, riprodotti a velocita doppia
    registra_video.py 60 1 /drone/camera/image_raw

Il file finisce in /ws/snapshots, montata su ./snapshots dell'host.
Ctrl-C chiude la registrazione in anticipo e salva quello che c'e.

La durata e in secondi SIMULATI e il video viene scritto in modo che un
secondo simulato duri un secondo di riproduzione (moltiplicato per la
velocita). Su una macchina lenta la simulazione gira a una frazione del tempo
reale e la telecamera perde fotogrammi: un fps fisso darebbe un video
accelerato e a scatti senza dirlo. Si salvano i fotogrammi con il loro istante
simulato e si calcola l'fps alla fine, da quelli davvero arrivati.

/target/debug_image si pubblica solo a missione avviata: in ATTESA il
rilevatore esce prima, e la registrazione resta in attesa del primo frame.
"""
import os
import sys
import tempfile
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String

DURATA_SIM = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
VELOCITA = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
TOPIC = sys.argv[3] if len(sys.argv) > 3 else '/target/debug_image'
CARTELLA = os.environ.get('CARTELLA_VIDEO', '/ws/snapshots')
# Oltre questo tempo senza fotogrammi si chiude comunque: senza, una
# simulazione ferma terrebbe aperta la registrazione per sempre.
SILENZIO_MAX_S = 60.0


class Registratore(Node):
    def __init__(self, cartella_frame):
        # Orologio simulato: serve quando l'immagine arriva senza istante.
        super().__init__('registra_video', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        sensore = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.bridge = CvBridge()
        self.cartella_frame = cartella_frame
        self.frame = []          # (istante simulato, percorso del jpg)
        self.dimensione = None
        self.fase = '?'
        self.quota = None
        self.ultimo_arrivo = time.monotonic()
        self.create_subscription(Image, TOPIC, self.su_immagine, sensore)
        self.create_subscription(String, '/mission/stato', self.su_fase, 10)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt',
                                 self.su_quota, sensore)

    def su_fase(self, msg):
        self.fase = msg.data

    def su_quota(self, msg):
        self.quota = msg.data

    def trascorso(self):
        if len(self.frame) < 2:
            return 0.0
        return self.frame[-1][0] - self.frame[0][0]

    def su_immagine(self, msg):
        istante = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # /target/debug_image esce con l'istante a zero: la durata risultava
        # sempre nulla e la registrazione non si fermava mai. In quel caso fa
        # fede l'orologio simulato, letto da /clock.
        if istante == 0.0:
            istante = self.get_clock().now().nanoseconds * 1e-9
        img =self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.dimensione is None:
            self.dimensione = (img.shape[1], img.shape[0])
        t_rel = istante - self.frame[0][0] if self.frame else 0.0
        quota = f'{self.quota:5.1f} m' if self.quota is not None else '  ? m'
        riga = f'{self.fase}   quota {quota}   t {t_rel:6.1f} s'
        # Fascia scura sotto il testo: leggibile anche sopra l'erba chiara.
        cv2.rectangle(img, (0, 0), (img.shape[1], 30), (0, 0, 0), -1)
        cv2.putText(img, riga, (10, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 1, cv2.LINE_AA)
        percorso = os.path.join(self.cartella_frame, f'{len(self.frame):06d}.jpg')
        cv2.imwrite(percorso, img, [cv2.IMWRITE_JPEG_QUALITY, 90])
        self.frame.append((istante, percorso))
        self.ultimo_arrivo = time.monotonic()
        if len(self.frame) % 25 == 0:
            print(f'  {len(self.frame)} frame, {self.trascorso():.1f} s simulati'
                  f' di {DURATA_SIM:.0f}  fase={self.fase}', flush=True)


def scrivi_video(frame, dimensione, destinazione):
    durata = frame[-1][0] - frame[0][0]
    fps = (len(frame) - 1) / durata * VELOCITA if durata > 0 else 10.0
    # L'fps va limitato: con pochi fotogrammi molto vicini nel tempo
    # simulato uscirebbe un valore assurdo che i lettori rifiutano.
    fps = max(1.0, min(fps, 60.0))
    scrittore = cv2.VideoWriter(destinazione, cv2.VideoWriter_fourcc(*'mp4v'),
                                fps, dimensione)
    if not scrittore.isOpened():
        raise RuntimeError(f'OpenCV non riesce a scrivere {destinazione}')
    for _, percorso in frame:
        scrittore.write(cv2.imread(percorso))
    scrittore.release()
    return fps, durata


def main():
    os.makedirs(CARTELLA, exist_ok=True)
    nome = time.strftime('demo_%Y%m%d_%H%M%S.mp4')
    destinazione = os.path.join(CARTELLA, nome)
    with tempfile.TemporaryDirectory(prefix='registra_video_') as tmp:
        rclpy.init()
        nodo = Registratore(tmp)
        print(f'Registro {TOPIC} per {DURATA_SIM:.0f} s simulati '
              f'(Ctrl-C per chiudere prima)...', flush=True)
        try:
            while rclpy.ok() and nodo.trascorso() < DURATA_SIM:
                rclpy.spin_once(nodo, timeout_sec=1.0)
                silenzio = time.monotonic() - nodo.ultimo_arrivo
                if not nodo.frame and silenzio > 15 and int(silenzio) % 15 == 0:
                    print('  ...nessun frame: la missione e avviata? '
                          'In ATTESA il rilevatore non pubblica.', flush=True)
                if silenzio > SILENZIO_MAX_S and nodo.frame:
                    print(f'  nessun frame da {SILENZIO_MAX_S:.0f} s: chiudo.',
                          flush=True)
                    break
        except KeyboardInterrupt:
            print('\n  interrotto: salvo quello che ho.', flush=True)
        finally:
            frame, dimensione = nodo.frame, nodo.dimensione
            nodo.destroy_node()
            rclpy.try_shutdown()

        if len(frame) < 2:
            print('Nessun video: ricevuti meno di due frame.', file=sys.stderr)
            sys.exit(1)
        fps, durata = scrivi_video(frame, dimensione, destinazione)
        print(f'Fatto: {destinazione}')
        print(f'  {len(frame)} frame, {durata:.1f} s simulati, {fps:.1f} fps, '
              f'dura {len(frame) / fps:.0f} s')


if __name__ == '__main__':
    main()
