#!/usr/bin/env bash
#
# Fotografa la configurazione attiva accanto ai dati della prova.
#
#   salva_config.sh [etichetta]
#
# Scrive in /ws/metrics un file <marca>_<etichetta>.params.yaml con i parametri
# di tutti i nodi del progetto, piu i parametri di ArduPilot che ci interessano
# se MAVProxy sta girando.
#
# A cosa serve. Il CSV di metrics_node dice come e andata la prova, non con che
# taratura: i parametri sono modificabili a caldo con `ros2 param set`, quindi
# rileggere i default nel codice a posteriori non dimostra nulla su una prova
# gia conclusa. Questo script chiude quella lacuna, ed e il motivo per cui le
# costanti tarate sono diventate parametri ROS.
#
set -e

source /opt/ros/jazzy/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash

CARTELLA="${CARTELLA_METRICHE:-/ws/metrics}"
mkdir -p "$CARTELLA"

ETICHETTA="${1:-}"
MARCA="$(date +%Y%m%d_%H%M%S)"
if [ -n "$ETICHETTA" ]; then
    USCITA="$CARTELLA/${MARCA}_${ETICHETTA}.params.yaml"
else
    USCITA="$CARTELLA/${MARCA}.params.yaml"
fi

NODI="detector_node tracker_node jammer_node controller_node mission_node
      target_mover_node metrics_node"

{
    echo "# Configurazione dei nodi al $(date -Is)"
    echo "# Prodotto da salva_config.sh — non modificare a mano."
    for nodo in $NODI; do
        echo ""
        echo "# --- /$nodo ---"
        if ! ros2 param dump "/$nodo" 2>/dev/null; then
            echo "# /$nodo non risponde (non avviato?)"
        fi
    done
} > "$USCITA"

echo "Configurazione salvata in $USCITA"

# I parametri dell'autopilota vivono in un altro sistema di configurazione e
# non si leggono con `ros2 param`. Si annota qui quali contano, perche una
# prova non e riproducibile conoscendo solo la meta ROS della taratura.
echo ""
echo "Parametri ArduPilot rilevanti (da leggere in MAVProxy con 'param show'):"
echo "  WP_SPD WP_ACC WP_JERK PSC_JERK_NE ATC_ANGLE_MAX ARMING_SKIPCHK"
echo "I default applicati all'avvio stanno in docker/sitl-defaults.parm."
