#!/usr/bin/env bash
#
# Sequenza di decollo via servizi MAVROS — equivalente dei comandi
# manuali in MAVProxy (mode guided / arm throttle / takeoff).
#
#   takeoff.sh [quota_metri]     default: 12 (quota di crociera dei waypoint)
#
# ARMING_CHECK=0 non si imposta qui: arriva da docker/sitl-defaults.parm, caricato
# all'avvio del SITL. Via MAVROS il set fallirebbe finché il pull dei parametri dal
# firmware non è completo, con un timing non prevedibile.
#
set -e

QUOTA="${1:-12.0}"

source /opt/ros/jazzy/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash

# Ogni chiamata va verificata: una risposta success=False è indistinguibile da un
# successo se si scarta l'output, ed è così che un decollo fallito passa
# inosservato fino a quando non ci si chiede perché il drone è ancora a terra.
chiama() {
    local descrizione="$1"; shift
    local esito
    esito="$(ros2 service call "$@" 2>&1 | tail -3)"
    # I servizi MAVROS non usano un campo comune: CommandBool e CommandTOL
    # rispondono con `success`, SetMode con `mode_sent`.
    if echo "$esito" | grep -qE 'success=True|mode_sent=True'; then
        return 0
    fi
    echo "   FALLITO: $descrizione" >&2
    echo "$esito" | sed 's/^/   /' >&2
    return 1
}

echo "→ Attendo che MAVROS sia connesso al SITL..."
until ros2 topic echo /mavros/state --once 2>/dev/null | grep -q "connected: true"; do
    sleep 1
done
echo "  connesso."

# Un decollo non parte se mission_node sta già pubblicando setpoint di posizione:
# in GUIDED quel flusso ha la precedenza sul comando di takeoff.
STATO="$(ros2 topic echo /mission/stato --field data --once 2>/dev/null | head -1 || true)"
if [ -n "$STATO" ] && [ "${STATO#ATTESA}" != "$STATO" ]; then
    : # in ATTESA, nessun setpoint in volo: tutto a posto
elif [ -n "$STATO" ]; then
    echo "ATTENZIONE: la missione è in stato '$STATO', non ATTESA." >&2
    echo "  mission_node sta già pubblicando setpoint di posizione e il decollo" >&2
    echo "  verrà ignorato. Riavvia lo stack prima di riprovare:" >&2
    echo "    docker compose restart && docker compose exec sim start_all.sh --detach" >&2
    exit 1
fi

echo "→ Modalità GUIDED"
chiama "set_mode GUIDED" /mavros/set_mode mavros_msgs/srv/SetMode \
    "{base_mode: 0, custom_mode: 'GUIDED'}"
sleep 2

echo "→ Arming"
chiama "arming" /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
sleep 2

echo "→ Decollo a ${QUOTA} m"
chiama "takeoff" /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL \
    "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: ${QUOTA}}"

# Conferma che il drone stia davvero salendo: i servizi possono rispondere
# success=True e il velivolo restare a terra.
echo "→ Verifico la salita..."
for _ in $(seq 1 30); do
    ALT="$(ros2 topic echo /mavros/global_position/rel_alt --field data --once 2>/dev/null | head -1)"
    if [ -n "$ALT" ] && awk "BEGIN{exit !($ALT > $QUOTA * 0.8)}"; then
        echo "  quota raggiunta: ${ALT} m"
        echo
        echo "Avvia la missione con:"
        echo "  ros2 topic pub --once /mission/avvia std_msgs/msg/Bool \"data: true\""
        exit 0
    fi
    sleep 2
done

echo "  ATTENZIONE: quota ferma a ${ALT:-?} m dopo 60 s." >&2
echo "  Controlla il pannello 'mavproxy' per il motivo del rifiuto." >&2
exit 1
