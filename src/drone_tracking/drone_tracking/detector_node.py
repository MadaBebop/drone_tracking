#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64, String
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from drone_tracking.mission_node import FaseMissione  # type: ignore
from drone_tracking.parametri import parametro  # type: ignore

class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')

        qos_gz = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber: feed telecamera reale dal drone
        self.camera_sub = self.create_subscription(
            Image, '/drone/camera/image_raw',
            self.on_image, qos_gz)

        # Subscriber: stato missione
        self.mission_sub = self.create_subscription(
            String, '/mission/stato',
            self.on_stato_missione, 10)

        # La quota serve a sapere quanto grande deve apparire il bersaglio.
        self.create_subscription(
            Float64, '/mavros/global_position/rel_alt', self.on_quota, qos_gz)
        self.quota = None

        # Soglia di validità del contorno, in pixel quadrati. Abbassata da 200
        # a 100 quando il FOV e passato a 90 gradi: la sfera occupa circa un
        # terzo dei pixel di prima a parita di quota.
        self.area_minima_px = parametro(self, 'area_minima_px', 100.0)

        # Estremi della maschera HSV per il rosso. La tinta del rosso sta a
        # cavallo dello zero, quindi servono due intervalli: [0, tolleranza] e
        # [180-tolleranza, 180]. Saturazione e valore minimi escludono i grigi e
        # le zone in ombra, che altrimenti rientrerebbero nella tinta giusta.
        self.tolleranza_tinta   = parametro(self, 'tolleranza_tinta', 10)
        self.saturazione_minima = parametro(self, 'saturazione_minima', 120)
        self.valore_minimo      = parametro(self, 'valore_minimo', 70)

        # Dimensioni note del bersaglio e semicampo dell'ottica: da questi e
        # dalla quota si ricava quanti pixel deve occupare. Un blob che si
        # discosta di piu di `tolleranza_area` volte non e quel bersaglio.
        self.lunghezza_bersaglio_m = parametro(self, 'lunghezza_bersaglio_m', 4.0)
        self.larghezza_bersaglio_m = parametro(self, 'larghezza_bersaglio_m', 2.0)
        # Quattro volte in piu o in meno: copre l'orientamento del veicolo e la
        # visione parziale al bordo dell'inquadratura, dove meta del bersaglio
        # e fuori campo e l'area dimezza.
        self.tolleranza_area = parametro(self, 'tolleranza_area', 4.0)
        self.tan_semi_fov_o = parametro(self, 'tan_semi_fov_o', 1.0)

        # Latenza iniettata fra rilevamento e pubblicazione, in secondi. A zero
        # il percorso resta quello diretto, senza coda ne timer.
        self.ritardo_s = parametro(self, 'ritardo_s', 0.0)
        self.coda_ritardo = []

        self.pos_smooth_x = 0.0
        self.pos_smooth_y = 0.0
        self.alpha = 1  # fattore smoothing (0=molto lento, 1=nessuno), rimosso per problemi di tracking ad alta velocità

        self.target_pub = self.create_publisher(
            PointStamped, '/target/position', 10)
        self.debug_pub  = self.create_publisher(Image, '/target/debug_image', 10)

        self.bridge = CvBridge()
        self.fase_missione = FaseMissione.ATTESA.value

        # Il timer esiste sempre ma non fa nulla a ritardo zero: crearlo solo
        # quando serve impedirebbe di accendere la latenza a caldo.
        self.create_timer(0.02, self._svuota_coda)

        self.get_logger().info('DetectorNode avviato — telecamera reale')

    def _pubblica_rilevamento(self, msg):
        """Pubblica subito, oppure mette in coda se la latenza e accesa."""
        if self.ritardo_s <= 0.0:
            self.target_pub.publish(msg)
            return
        quando = self.get_clock().now().nanoseconds / 1e9 + self.ritardo_s
        self.coda_ritardo.append((quando, msg))

    def _svuota_coda(self):
        if not self.coda_ritardo:
            return
        adesso = self.get_clock().now().nanoseconds / 1e9
        pronti = [c for c in self.coda_ritardo if c[0] <= adesso]
        self.coda_ritardo = [c for c in self.coda_ritardo if c[0] > adesso]
        for _, msg in pronti:
            self.target_pub.publish(msg)

    def on_stato_missione(self, msg: String):
        self.fase_missione = msg.data

    def on_quota(self, msg: Float64):
        self.quota = msg.data

    def _area_attesa(self, larghezza_px):
        """Pixel quadrati che il bersaglio deve occupare alla quota corrente.

        A quota h l'impronta a terra vale 2*h*tan(semicampo), quindi la scala e
        (larghezza_px/2) / (h*tan(semicampo)) pixel per metro. L'area cresce con
        il quadrato della scala, cioe cala con 1/h^2.
        """
        if self.quota is None or self.quota < 2.0:
            return None
        px_per_m = (larghezza_px / 2.0) / (self.quota * self.tan_semi_fov_o)
        return (self.lunghezza_bersaglio_m * self.larghezza_bersaglio_m
                * px_per_m * px_per_m)

    def _taglia_plausibile(self, area, larghezza_px):
        """Vero se l'area e compatibile con un bersaglio a questa quota.

        Senza quota nota non si puo giudicare, e si accetta: e il comportamento
        precedente, ed e preferibile a scartare per ignoranza.
        """
        attesa = self._area_attesa(larghezza_px)
        if attesa is None:
            return True
        return attesa / self.tolleranza_area <= area <= attesa * self.tolleranza_area

    def on_image(self, msg: Image):
        point_msg = PointStamped()
        # L'istante e quello dell'otturatore, non quello della pubblicazione:
        # fra i due c'e il ponte della telecamera, che e latenza quanto il
        # resto. Chi riceve deve sapere quando la scena e stata guardata.
        point_msg.header = msg.header

        # Non rilevare in ATTESA
        if FaseMissione.ATTESA.value in self.fase_missione:
            self._pubblica_rilevamento(point_msg)
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Rilevamento colore rosso in HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        t, s_min, v_min = (self.tolleranza_tinta, self.saturazione_minima,
                           self.valore_minimo)
        mask1 = cv2.inRange(hsv, np.array([0, s_min, v_min]),
                            np.array([t, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([180 - t, s_min, v_min]),
                            np.array([180, 255, 255]))
        mask = mask1 | mask2

        # Estrazione dei contorni del target
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Estrae il contorno con l'area maggiore
            c = max(contours, key=cv2.contourArea)
            area_corrente = cv2.contourArea(c)
            
            # Sotto la soglia si tratta di rumore visivo, non del bersaglio.
            larghezza_px = frame.shape[1]
            plausibile = self._taglia_plausibile(area_corrente, larghezza_px)
            if area_corrente > self.area_minima_px and not plausibile:
                self.get_logger().warn(
                    'Blob da {:.0f} px scartato: a {:.0f} m ne servono ~{:.0f}'
                    .format(area_corrente, self.quota or 0.0,
                            self._area_attesa(larghezza_px) or 0.0),
                    throttle_duration_sec=5.0)

            if area_corrente > self.area_minima_px and plausibile:
                M = cv2.moments(c)
                
                # Protezione da divisione per zero (può capitare se l'area del momento M['m00'] è nulla)
                if M['m00'] > 0:
                    px = int(M['m10'] / M['m00'])
                    py = int(M['m01'] / M['m00'])

                    h, w = frame.shape[:2]
                    
                    # Coordinate geometriche normalizzate nell'intervallo [-1.0, 1.0]
                    raw_x = (px - w / 2) / (w / 2)
                    raw_y = (py - h / 2) / (h / 2)
                    
                    # Smoothing esponenziale (con alpha=1 passa il dato puro senza lag)
                    self.pos_smooth_x = self.alpha * raw_x + (1 - self.alpha) * self.pos_smooth_x
                    self.pos_smooth_y = self.alpha * raw_y + (1 - self.alpha) * self.pos_smooth_y
                    
                    point_msg.point.x = self.pos_smooth_x
                    point_msg.point.y = self.pos_smooth_y
                    point_msg.point.z = float(area_corrente) # Flag di visibilità cruciale (z > 0 significa presente)

                    # Disegni grafici di debug sul frame per monitorare il tracking
                    cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
                    cv2.circle(frame, (px, py), 5, (255, 255, 0), -1)
                    cv2.putText(frame,
                                f'x:{point_msg.point.x:.2f} y:{point_msg.point.y:.2f} z:{point_msg.point.z:.0f}',
                                (px + 10, py),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                else:
                    # Momento non calcolabile, bersaglio non valido
                    point_msg.point.x = 0.0
                    point_msg.point.y = 0.0
                    point_msg.point.z = 0.0
            else:
                # Contorno troppo piccolo, azzera la pos e forza l'area a 0
                point_msg.point.x = 0.0
                point_msg.point.y = 0.0
                point_msg.point.z = 0.0
        else:
            # Nessun contorno rilevato, bersaglio perso
            point_msg.point.x = 0.0
            point_msg.point.y = 0.0
            point_msg.point.z = 0.0

        # Pubblicazione sui topic di ROS 2. L'immagine di debug non passa dalla
        # coda: serve a guardare cosa vede il rilevatore adesso, non a simulare
        # cosa sapeva il controllo un decimo di secondo fa.
        self._pubblica_rilevamento(point_msg)
        debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.debug_pub.publish(debug_msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()