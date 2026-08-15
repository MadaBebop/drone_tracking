#!/usr/bin/env bash
#
# Salva i frame annotati di /target/debug_image come PNG, per ispezionare il
# rilevamento senza GUI. I file finiscono in /ws/snapshots, che docker-compose
# monta su ./snapshots sull'host: apribili direttamente da Esplora Risorse.
#
#   snapshot.sh [numero_frame] [topic]
#
# Esempi:
#   snapshot.sh              # 20 frame da /target/debug_image
#   snapshot.sh 50           # 50 frame
#   snapshot.sh 10 /drone/camera/image_raw   # feed grezzo, senza annotazioni
#
set -e

N="${1:-20}"
TOPIC="${2:-/target/debug_image}"
OUTDIR=/ws/snapshots

source /opt/ros/jazzy/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash

mkdir -p "$OUTDIR"

echo "Salvo $N frame da $TOPIC in $OUTDIR ..."

python3 - "$N" "$TOPIC" "$OUTDIR" <<'PY'
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

n_target = int(sys.argv[1])
topic    = sys.argv[2]
outdir   = sys.argv[3]

class Snapper(Node):
    def __init__(self):
        super().__init__('snapshot_node')
        # Il feed della camera arriva con QoS sensor-like: BEST_EFFORT è
        # obbligatorio, un subscriber RELIABLE non riceverebbe nulla.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.bridge = CvBridge()
        self.count = 0
        self.create_subscription(Image, topic, self.on_image, qos)

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        path = f'{outdir}/frame_{self.count:04d}.png'
        cv2.imwrite(path, frame)
        self.count += 1
        print(f'  [{self.count}/{n_target}] {path}', flush=True)

rclpy.init()
node = Snapper()
try:
    while rclpy.ok() and node.count < n_target:
        rclpy.spin_once(node, timeout_sec=5.0)
        if node.count == 0:
            print('  ...nessun frame ricevuto: il topic pubblica? '
                  'La missione è avviata (in ATTESA il detector non pubblica)?',
                  flush=True)
            break
finally:
    node.destroy_node()
    rclpy.shutdown()

print(f'Fatto: {node.count} frame salvati in {outdir}')
PY
