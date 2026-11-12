#!/usr/bin/env bash
#
# Da eseguire SULLA VM Ubuntu dove il progetto gira nativamente.
# Copia dentro il repo i soli asset di simulazione personalizzati
# (mondo con il bersaglio rosso, modello del drone con camera a 45°),
# che upstream ardupilot_gazebo non contiene.
#
#   ./scripts/export_sim_assets.sh "/home/mada/Desktop/Progetto Drone/ardupilot_gazebo"
#
set -e

SRC="${1:-$HOME/Desktop/Progetto Drone/ardupilot_gazebo}"
DEST="$(cd "$(dirname "$0")/.." && pwd)/sim"

if [ ! -d "$SRC" ]; then
    echo "ERRORE: sorgente non trovata: $SRC" >&2
    echo "Uso: $0 /percorso/di/ardupilot_gazebo" >&2
    exit 1
fi

mkdir -p "$DEST/worlds" "$DEST/models"

echo "Sorgente:    $SRC"
echo "Destinazione: $DEST"
echo

# --- Mondo con la sfera "bersaglio" ---
if [ -f "$SRC/worlds/iris_runway.sdf" ]; then
    cp -v "$SRC/worlds/iris_runway.sdf" "$DEST/worlds/"
else
    echo "ATTENZIONE: worlds/iris_runway.sdf non trovato" >&2
fi

# --- Modelli del drone (quello con la camera è iris_with_ardupilot) ---
for m in iris_with_ardupilot iris_with_standoffs bersaglio; do
    if [ -d "$SRC/models/$m" ]; then
        cp -rv "$SRC/models/$m" "$DEST/models/"
    fi
done

echo
echo "Fatto. Ora dall'host con Docker:"
echo "  docker compose build"
