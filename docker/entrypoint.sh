#!/usr/bin/env bash
# Entrypoint: prepara l'ambiente ROS 2 e passa il controllo al comando richiesto.
set -e

source /opt/ros/jazzy/setup.bash
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi

# Gazebo scrive qui il proprio stato: senza HOME scrivibile il server non parte.
export HOME="${HOME:-/root}"
mkdir -p "$HOME/.gz"

exec "$@"
