#!/usr/bin/env bash
#
# Avvia l'intero stack in una sessione tmux, un pannello per componente —
# stesso ordine dei 5 terminali usati a mano sulla VM.
#
#   start_all.sh            avvia e si aggancia alla sessione
#   start_all.sh --detach   avvia in background
#   tmux attach -t drone    per riagganciarsi in seguito
#   tmux kill-session -t drone  per fermare tutto
#
set -e

SESSION=drone
WORLD_FILE="${WORLD:-iris_runway.sdf}"
WORLD_PATH="/opt/ardupilot_gazebo/worlds/${WORLD_FILE}"

if [ ! -f "$WORLD_PATH" ]; then
    echo "ERRORE: mondo non trovato: $WORLD_PATH" >&2
    echo "Mondi disponibili:" >&2
    ls -1 /opt/ardupilot_gazebo/worlds/ >&2
    exit 1
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Sessione '$SESSION' già attiva — mi aggancio."
    exec tmux attach -t "$SESSION"
fi

# Con HEADLESS=1 gira solo il server fisico (nessuna finestra, nessun X11).
if [ "${HEADLESS:-1}" = "1" ]; then
    GZ_ARGS="-s -v4 -r"
else
    GZ_ARGS="-v4 -r"
fi

SOURCES='source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash'

echo "Mondo:    $WORLD_PATH"
echo "Headless: ${HEADLESS:-1}"

# --- T1: Gazebo ---
tmux new-session  -d -s "$SESSION" -n gazebo \
    "gz sim $GZ_ARGS '$WORLD_PATH'; exec bash"

# --- T2: ArduPilot SITL (attende che Gazebo apra le porte JSON) ---
tmux new-window -t "$SESSION" -n sitl \
    "sleep 8; cd /opt/ardupilot && \
     ./build/sitl/bin/arducopter --model JSON --speedup 1 \
        --sim-address=127.0.0.1 --sim-port-in=9013 --sim-port-out=9012 \
        -I0 --defaults Tools/autotest/default_params/gazebo-iris.parm,/opt/sitl-defaults.parm; exec bash"

# --- T3: MAVProxy — hub MAVLink e console per i comandi manuali ---
tmux new-window -t "$SESSION" -n mavproxy \
    "sleep 16; mavproxy.py --master tcp:127.0.0.1:5760 \
        --out 127.0.0.1:14550 --out udp:127.0.0.1:14555; exec bash"

# --- T4: MAVROS2 ---
# Collegamento diretto alla porta TCP SERIAL1 del SITL, non all'uscita UDP di
# MAVProxy. UDP perde pacchetti e il relay aggiunge latenza: le conseguenti
# perdite di heartbeat innescano un bug di MAVROS, che risolve due volte la
# stessa promise sulla richiesta AUTOPILOT_VERSION e aborta con
# "std::future_error: Promise already satisfied". Il TCP e ordinato e senza
# perdite. MAVProxy resta sulla 5760 per i comandi manuali.
tmux new-window -t "$SESSION" -n mavros \
    "sleep 22; $SOURCES && \
     ros2 run mavros mavros_node --ros-args \
        -p fcu_url:=tcp://127.0.0.1:5762 \
        -p target_system_id:=1 -p target_component_id:=1 \
        -p plugin_denylist:=[distance_sensor]; exec bash"

# --- T5: nodi del progetto + ponte immagini ros_gz ---
tmux new-window -t "$SESSION" -n nodes \
    "sleep 30; $SOURCES && \
     ros2 launch drone_tracking tracking.launch.py; exec bash"

# --- T6: cruscotto live della missione ---
tmux new-window -t "$SESSION" -n monitor \
    "sleep 34; $SOURCES && monitor.sh; exec bash"

# --- T7: shell libera per takeoff.sh, ros2 topic echo, ... ---
tmux new-window -t "$SESSION" -n shell \
    "$SOURCES && exec bash"

tmux select-window -t "$SESSION":mavproxy

cat <<'EOF'

Stack avviato. Lo startup completo richiede ~30 s (i pannelli partono scaglionati).

  Ctrl-b n / p   pannello successivo / precedente
  Ctrl-b d       stacca la sessione (i processi continuano)
  tmux attach -t drone        per riagganciarsi
  tmux kill-session -t drone  per fermare tutto

Dal pannello 'shell':
  takeoff.sh                                                    # arm + guided + decollo a 12 m
  ros2 topic pub --once /mission/avvia std_msgs/msg/Bool "data: true"

Poi passa al pannello 'monitor' per seguire la missione in tempo reale.

EOF

if [ "$1" = "--detach" ]; then
    exit 0
fi

exec tmux attach -t "$SESSION"
