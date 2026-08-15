#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from drone_tracking.mission_node import FaseMissione  # type: ignore

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

        self.pos_smooth_x = 0.0
        self.pos_smooth_y = 0.0
        self.alpha = 1  # fattore smoothing (0=molto lento, 1=nessuno), rimosso per problemi di tracking ad alta velocità

        self.target_pub = self.create_publisher(Point, '/target/position', 10)
        self.debug_pub  = self.create_publisher(Image, '/target/debug_image', 10)

        self.bridge = CvBridge()
        self.fase_missione = FaseMissione.ATTESA.value

        self.get_logger().info('DetectorNode avviato — telecamera reale')

    def on_stato_missione(self, msg: String):
        self.fase_missione = msg.data

    def on_image(self, msg: Image):
        point_msg = Point()

        # Non rilevare in ATTESA
        if FaseMissione.ATTESA.value in self.fase_missione:
            self.target_pub.publish(point_msg)
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Rilevamento colore rosso in HSV
        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, np.array([0, 120, 70]),   np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        mask  = mask1 | mask2

        # Estrazione dei contorni del target
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Estrae il contorno con l'area maggiore
            c = max(contours, key=cv2.contourArea)
            area_corrente = cv2.contourArea(c)
            
            # Soglia di validità del contorno (evita il rumore visivo)
            # Soglia abbassata da 200 a 100 px^2: con il FOV allargato a 90° la
            # sfera occupa circa un terzo dei pixel di prima a parita di quota.
            if area_corrente > 100:
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
                    
                    point_msg.x = self.pos_smooth_x
                    point_msg.y = self.pos_smooth_y
                    point_msg.z = float(area_corrente) # Flag di visibilità cruciale (z > 0 significa presente)

                    # Disegni grafici di debug sul frame per monitorare il tracking
                    cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
                    cv2.circle(frame, (px, py), 5, (255, 255, 0), -1)
                    cv2.putText(frame,
                                f'x:{point_msg.x:.2f} y:{point_msg.y:.2f} z:{point_msg.z:.0f}',
                                (px + 10, py),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                else:
                    # Momento non calcolabile, bersaglio non valido
                    point_msg.x = 0.0
                    point_msg.y = 0.0
                    point_msg.z = 0.0
            else:
                # Contorno troppo piccolo, azzera la pos e forza l'area a 0
                point_msg.x = 0.0
                point_msg.y = 0.0
                point_msg.z = 0.0
        else:
            # Nessun contorno rilevato, bersaglio perso
            point_msg.x = 0.0
            point_msg.y = 0.0
            point_msg.z = 0.0

        # Pubblicazione sui topic di ROS 2
        self.target_pub.publish(point_msg)
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