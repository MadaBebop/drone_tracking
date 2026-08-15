#!/usr/bin/env bash
#
# Cruscotto live della missione: fase, telemetria del drone, stato del jamming e
# la catena di percezione nei suoi tre stadi, su una sola schermata.
#
#   monitor.sh [intervallo_secondi]     default: 0.5
#
# Ctrl-C per uscire.
#
set -e

source /opt/ros/jazzy/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash

exec python3 - "${1:-0.5}" <<'PY'
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool, String, Float32, Float64
from geometry_msgs.msg import Point, PoseStamped, Twist
from sensor_msgs.msg import Image

REFRESH = float(sys.argv[1])

# Colori ANSI: la leggibilità a colpo d'occhio è tutto il punto di questo script.
R = '\033[0m'; B = '\033[1m'; DIM = '\033[2m'
RED = '\033[31m'; GRN = '\033[32m'; YEL = '\033[33m'; CYA = '\033[36m'

COLORE_FASE = {
    'ATTESA':         DIM,
    'PATTUGLIAMENTO': CYA,
    'AGGANCIO':       GRN,
    'RICERCA':        YEL,
}


def barra(x, larghezza=41):
    """Posizione orizzontale nel frame, da -1 (sinistra) a +1 (destra)."""
    if x is None:
        return DIM + '·' * larghezza + R
    centro = larghezza // 2
    pos = int(round((max(-1.0, min(1.0, x)) + 1.0) / 2.0 * (larghezza - 1)))
    celle = ['·'] * larghezza
    celle[centro] = '|'
    celle[pos] = '#'
    return ''.join(celle)


class Monitor(Node):
    def __init__(self):
        super().__init__('monitor_node')

        # MAVROS e la camera pubblicano con QoS sensor-like: un subscriber
        # RELIABLE non riceverebbe nulla.
        best = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=1)

        self.fase = '—'
        self.alt = None
        self.pos = None
        self.jammed = False
        self.noise = 0.0
        self.raw = self.jam = self.trk = None
        self.cmd = None
        self.frame_ts = deque(maxlen=30)

        self.create_subscription(String, '/mission/stato',
                                 lambda m: setattr(self, 'fase', m.data), 10)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt',
                                 lambda m: setattr(self, 'alt', m.data), best)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 lambda m: setattr(self, 'pos', m.pose.position), best)
        self.create_subscription(Bool, '/gps/jammed',
                                 lambda m: setattr(self, 'jammed', m.data), 10)
        self.create_subscription(Float32, '/rf/noise_level',
                                 lambda m: setattr(self, 'noise', m.data), 10)
        self.create_subscription(Point, '/target/position',
                                 lambda m: setattr(self, 'raw', m), 10)
        self.create_subscription(Point, '/target/jammed_position',
                                 lambda m: setattr(self, 'jam', m), 10)
        self.create_subscription(Point, '/target/tracked_position',
                                 lambda m: setattr(self, 'trk', m), 10)
        self.create_subscription(Twist, '/drone/cmd_vel',
                                 lambda m: setattr(self, 'cmd', m), 10)
        self.create_subscription(Image, '/drone/camera/image_raw',
                                 lambda m: self.frame_ts.append(time.time()), best)

    def hz(self):
        if len(self.frame_ts) < 2:
            return 0.0
        span = self.frame_ts[-1] - self.frame_ts[0]
        return (len(self.frame_ts) - 1) / span if span > 0 else 0.0

    @staticmethod
    def _xy(p):
        """None se il bersaglio non è disponibile.

        Si usa il criterio `x != 0 or y != 0`, lo stesso di controller_node e
        mission_node, e NON `z != 0`. Motivo: durante una perdita di segnale
        tracker_node pubblica la predizione di Kalman ricopiando `z` dal
        messaggio in ingresso, che vale 0 — la predizione è valida ma verrebbe
        letta come assenza.
        """
        if p is None or (p.x == 0.0 and p.y == 0.0):
            return None, None
        return p.x, p.y

    def render(self):
        fase_base = self.fase.split(':')[0]
        col = COLORE_FASE.get(fase_base, '')

        alt = f'{self.alt:5.1f} m' if self.alt is not None else '    — '
        if self.pos is not None:
            xy = f'x{self.pos.x:6.1f}  y{self.pos.y:6.1f}'
        else:
            xy = '     —         — '

        if self.jammed:
            jam_txt = f'{RED}{B}JAMMING ATTIVO{R}  rumore RF {self.noise:.1f}'
        else:
            jam_txt = f'{GRN}segnale pulito{R}  rumore RF {self.noise:.1f}'

        rx, _ = self._xy(self.raw)
        jx, _ = self._xy(self.jam)
        tx, _ = self._xy(self.trk)

        def val(v):
            return f'{v:+.3f}' if v is not None else ' assente'

        vy = f'{self.cmd.linear.y:+.2f}' if self.cmd else ' 0.00'
        vx = f'{self.cmd.linear.x:+.2f}' if self.cmd else ' 0.00'

        out = [
            f'{B}  DRONE TRACKING — monitor{R}   {DIM}Ctrl-C per uscire{R}',
            '',
            f'  Fase        {col}{B}{self.fase:<22}{R}',
            f'  Quota       {alt}        Posizione   {xy}',
            f'  Camera      {self.hz():4.1f} Hz',
            f'  Datalink    {jam_txt}',
            '',
            f'{B}  Catena di percezione{R}  {DIM}(posizione orizzontale nel frame){R}',
            f'  detector    {val(rx)}  {barra(rx)}',
            f'  jammer      {val(jx)}  {barra(jx)}',
            f'  kalman      {val(tx)}  {GRN}{barra(tx)}{R}',
            '',
            f'  Comando     laterale {vy} m/s   longitudinale {vx} m/s',
        ]
        # \033[H porta il cursore in alto, \033[J pulisce da lì in giù: evita lo
        # sfarfallio di un clear completo a ogni ciclo.
        print('\033[H\033[J' + '\n'.join(out), flush=True)


rclpy.init()
node = Monitor()
try:
    prossimo = time.time()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() >= prossimo:
            node.render()
            prossimo = time.time() + REFRESH
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
    print()
PY
