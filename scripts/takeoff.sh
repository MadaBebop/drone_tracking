#!/usr/bin/env bash
#
# Sequenza di decollo via servizi MAVROS — equivalente dei comandi
# manuali in MAVProxy (param set / mode guided / arm throttle / takeoff).
#
#   takeoff.sh [quota_metri]     default: 12 (quota di crociera dei waypoint)
#
set -e

QUOTA="${1:-12.0}"

source /opt/ros/jazzy/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash

echo "→ Attendo che MAVROS sia connesso al SITL..."
until ros2 topic echo /mavros/state --once 2>/dev/null | grep -q "connected: true"; do
    sleep 1
done
echo "  connesso."

# ARMING_CHECK=0 non si imposta qui: arriva da docker/sitl-defaults.parm, caricato
# all'avvio del SITL. Via MAVROS il set fallirebbe finché il pull dei parametri dal
# firmware non è completo, con un timing non prevedibile.

echo "→ Modalità GUIDED"
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
    "{base_mode: 0, custom_mode: 'GUIDED'}" > /dev/null
sleep 2

echo "→ Arming"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \
    "{value: true}" > /dev/null
sleep 2

echo "→ Decollo a ${QUOTA} m"
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL \
    "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: ${QUOTA}}" > /dev/null

echo
echo "Decollo comandato. Verifica la quota con:"
echo "  ros2 topic echo /mavros/global_position/rel_alt"
echo
echo "Quando il drone è in quota, avvia la missione con:"
echo "  ros2 topic pub --once /mission/avvia std_msgs/msg/Bool \"data: true\""
