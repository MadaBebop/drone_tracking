#!/usr/bin/env bash
#
# Sequenza di decollo via servizi MAVROS — equivalente dei comandi
# manuali in MAVProxy (mode guided / arm throttle / takeoff).
#
#   takeoff.sh [quota_metri]     default: 50 (quota di crociera dei waypoint)
#
# I controlli di arming non si disattivano qui: ARMING_SKIPCHK=1 arriva da
# docker/sitl-defaults.parm, caricato all'avvio del SITL. Via MAVROS il set
# fallirebbe finché il pull dei parametri dal firmware non è completo, con un
# timing non prevedibile. Attenzione al nome: ARMING_CHECK non esiste in questa
# versione di ArduPilot e verrebbe ignorato in silenzio.
#
set -e

QUOTA="${1:-50.0}"

source /opt/ros/jazzy/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash

# --- Lettura di un topic, sempre terminante ---------------------------------
# `ros2 topic echo --once` puo bloccarsi per sempre: se la QoS della
# sottoscrizione non combacia con quella del publisher resta in attesa di un
# messaggio che non arrivera mai. E successo davvero, e una prova e rimasta
# ferma tre ore e quaranta con il drone in volo e nessuno che se ne accorgesse.
# Il timeout la rende terminante; best_effort combacia con qualunque publisher,
# affidabile o sensor-like che sia.
leggi_topic() {
    local topic="$1" campo="$2" limite="${3:-15}"
    if [ -n "$campo" ]; then
        timeout "$limite" ros2 topic echo --once --qos-reliability best_effort \
            --field "$campo" "$topic" 2>/dev/null | head -1
    else
        timeout "$limite" ros2 topic echo --once --qos-reliability best_effort \
            "$topic" 2>/dev/null
    fi
}

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
for _ in $(seq 1 60); do
    leggi_topic /mavros/state "" 10 | grep -q "connected: true" && break
    sleep 1
done
if ! leggi_topic /mavros/state "" 10 | grep -q "connected: true"; then
    echo "   ERRORE: MAVROS non risulta connesso al SITL." >&2
    echo "   Controlla il pannello 'mavros' e quello 'mavproxy'." >&2
    exit 1
fi
echo "  connesso."

# Un decollo non parte se mission_node sta già pubblicando setpoint di posizione:
# in GUIDED quel flusso ha la precedenza sul comando di takeoff.
STATO="$(leggi_topic /mission/stato data 10)"
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

# L'arming puo essere rifiutato in modo transitorio: se la fisica ha singhiozzato,
# gli IMU simulati risultano incoerenti per qualche secondo. Si riprova.
echo "→ Arming"
for tentativo in 1 2 3 4 5; do
    if chiama "arming (tentativo $tentativo)" /mavros/cmd/arming             mavros_msgs/srv/CommandBool "{value: true}"; then
        break
    fi
    if [ "$tentativo" = 5 ]; then
        echo "   arming rifiutato 5 volte: controlla il pannello 'mavproxy'." >&2
        exit 1
    fi
    sleep 3
done
sleep 2

echo "→ Decollo a ${QUOTA} m"
chiama "takeoff" /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL \
    "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: ${QUOTA}}"

# Conferma che il drone stia davvero salendo: i servizi possono rispondere
# success=True e il velivolo restare a terra.
# La salita a 50 m richiede una decina di secondi simulati, che su una macchina
# senza accelerazione grafica valgono quaranta secondi di orologio: il limite
# va tenuto largo, altrimenti il decollo viene dichiarato fallito mentre e
# ancora in corso.
echo "→ Verifico la salita..."
for _ in $(seq 1 60); do
    ALT="$(leggi_topic /mavros/global_position/rel_alt data 10)"
    if [ -n "$ALT" ] && awk "BEGIN{exit !($ALT > $QUOTA * 0.8)}"; then
        echo "  quota raggiunta: ${ALT} m"
        echo
        echo "Avvia la missione con:"
        echo "  ros2 topic pub --once /mission/avvia std_msgs/msg/Bool \"data: true\""
        exit 0
    fi
    sleep 2
done

echo "  ATTENZIONE: quota ferma a ${ALT:-?} m dopo 120 s." >&2
echo "  Controlla il pannello 'mavproxy' per il motivo del rifiuto." >&2
exit 1
