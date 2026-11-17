#!/usr/bin/env bash
#
# Demo in un comando: dallo stack appena avviato al drone che pattuglia e
# insegue il bersaglio, con eventuale registrazione video.
#
#   demo.sh [secondi_video]       default: nessun video
#
# Dall'host, con lo stack gia avviato da start_all.sh --detach:
#   docker compose exec sim demo.sh
#   docker compose exec sim demo.sh 150     # e registra 150 s simulati
#
# Esiste per il portatile lento, dove la sequenza a mano fallisce in due modi
# entrambi gia visti:
#  - il decollo subito dopo l'avvio viene rifiutato con "Gyros inconsistent"
#    finche la simulazione, ancora sotto carico, non si e assestata. I cinque
#    tentativi di takeoff.sh si esauriscono in mezzo minuto: qui si aspetta
#    prima e, se serve, si ripete takeoff.sh intero dopo una pausa;
#  - i nodi e MAVROS impiegano piu dei 30 secondi previsti da start_all.sh.
#
#  - sotto carico la telemetria di MAVROS si ferma per qualche secondo, e il
#    nodo di sicurezza, che giudica guasto un silenzio oltre 1.5 s, interrompe
#    la missione e comanda RTL. Su una macchina sana e la scelta giusta; qui e
#    un artefatto del portatile, quindi per la demo la soglia si allarga.
#
# Con la stazione a terra (start_all.sh lanciato con LAUNCH_ARGS="stazione:=true")
# demo.sh non decolla e non avvia la missione: lo fa la stazione alla conferma
# dell'allerta, e lo script aspetta e poi registra.
#
# Variabili d'ambiente:
#   DEMO_ASSESTAMENTO       secondi di attesa prima del decollo (default 60)
#   DEMO_GIRI_DECOLLO       quante volte ripetere takeoff.sh (default 3)
#   DEMO_TIMEOUT_TELEMETRIA soglia del nodo di sicurezza, in s (default 5.0;
#                           1.5 per lasciare quella del progetto)
#   DEMO_FUGA_S             durata della fuga del bersaglio con la stazione
#                           (default 8.0; 20.0 per lasciare quella del progetto)
#
set -e

VIDEO_S="${1:-0}"
ASSESTAMENTO="${DEMO_ASSESTAMENTO:-60}"
GIRI="${DEMO_GIRI_DECOLLO:-3}"
TIMEOUT_TELEMETRIA="${DEMO_TIMEOUT_TELEMETRIA:-5.0}"
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"

source /opt/ros/jazzy/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash

# Stessa lettura terminante di takeoff.sh e prova.sh: `ros2 topic echo --once`
# senza timeout puo restare appeso per sempre se la QoS non combacia.
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

ora() { date +%H:%M:%S; }

if ! tmux has-session -t drone 2>/dev/null; then
    echo "ERRORE: lo stack non e avviato. Prima:" >&2
    echo "  docker compose exec sim start_all.sh --detach" >&2
    exit 1
fi

echo "[$(ora)] → Attendo i 6 nodi del progetto"
for _ in $(seq 1 60); do
    N="$(timeout 20 ros2 node list 2>/dev/null \
        | grep -cE '(detector|tracker|jammer|controller|mission|target_mover)_node' || true)"
    [ "$N" = 6 ] && break
    sleep 3
done
if [ "$N" != 6 ]; then
    echo "   ERRORE: attivi $N nodi su 6. Controlla con: tmux attach -t drone" >&2
    exit 1
fi
echo "  ok."

echo "[$(ora)] → Soglia di telemetria del nodo di sicurezza: ${TIMEOUT_TELEMETRIA} s"
if ! ros2 param set /sicurezza_node timeout_telemetria_s "$TIMEOUT_TELEMETRIA" >/dev/null 2>&1; then
    echo "   ATTENZIONE: non riesco a impostarla, resta quella del progetto." >&2
fi

echo "[$(ora)] → Attendo MAVROS collegato al simulatore"
for _ in $(seq 1 60); do
    leggi_topic /mavros/state "" 10 | grep -q "connected: true" && break
    sleep 2
done
if ! leggi_topic /mavros/state "" 10 | grep -q "connected: true"; then
    echo "   ERRORE: MAVROS non collegato. Controlla i pannelli 'mavros' e 'mavproxy'." >&2
    exit 1
fi
echo "  ok."

if timeout 20 ros2 node list 2>/dev/null | grep -q 'stazione_node'; then
    # Scenario con la stazione a terra: decollo e avvio li decide lei, alla
    # conferma dell'allerta. Qui si aspetta soltanto, e si racconta.
    #
    # La fuga del bersaglio si accorcia: con quella di default (20 s a 15 m/s,
    # quasi 300 m) il bersaglio si stabiliva a 460-540 m dalla stazione, oltre
    # il confine di sicurezza dei 500 m, e la demo finiva con il drone
    # riportato a casa dal livello di sicurezza. E una scelta di scenario, e
    # va dichiarata come tale.
    FUGA="${DEMO_FUGA_S:-8.0}"
    echo "[$(ora)] → Scenario della demo: fuga del bersaglio di ${FUGA} s"
    # Subito dopo l'avvio il servizio dei parametri puo non rispondere in
    # tempo: alla prima prova il comando e fallito e la fuga e rimasta di 20 s.
    # Si ritenta, e si verifica rileggendo invece di fidarsi dell'esito.
    for _ in 1 2 3 4 5; do
        ros2 param set /target_mover_node durata_evasione_s "$FUGA" >/dev/null 2>&1 || true
        LETTA="$(timeout 20 ros2 param get /target_mover_node durata_evasione_s 2>/dev/null \
            | awk '{print $NF}')"
        if [ -n "$LETTA" ] && awk "BEGIN{exit !($LETTA == $FUGA)}"; then
            break
        fi
        sleep 5
    done
    if [ -z "$LETTA" ] || ! awk "BEGIN{exit !($LETTA == $FUGA)}"; then
        echo "   ATTENZIONE: fuga non impostata (letto: ${LETTA:-niente})." >&2
    fi
    echo "[$(ora)] → Stazione a terra attiva: attendo che lanci il drone"
    PRECEDENTE=""
    for _ in $(seq 1 120); do
        STAZ="$(leggi_topic /stazione/stato data 10)"
        FASE="$(leggi_topic /mission/stato data 10)"
        if [ -n "$STAZ" ] && [ "$STAZ" != "$PRECEDENTE" ]; then
            echo "  stazione: $STAZ"
            PRECEDENTE="$STAZ"
        fi
        if [ "$STAZ" = ERRORE ]; then
            echo "   ERRORE: lancio fallito. Motivo nel pannello 'nodes'." >&2
            exit 1
        fi
        case "$FASE" in ''|ATTESA*) sleep 5 ;; *) break ;; esac
    done
    echo "[$(ora)] → Missione avviata dalla stazione, fase: $FASE"
elif ALT="$(leggi_topic /mavros/global_position/rel_alt data 10)" \
        && [ -n "$ALT" ] && awk "BEGIN{exit !($ALT > 5.0)}"; then
    echo "[$(ora)] → Il drone e gia in volo a ${ALT} m: salto il decollo"
    ros2 topic pub --once /mission/avvia std_msgs/msg/Bool "data: true" >/dev/null
else
    echo "[$(ora)] → Lascio assestare la simulazione per ${ASSESTAMENTO} s"
    sleep "$ASSESTAMENTO"
    for giro in $(seq 1 "$GIRI"); do
        echo "[$(ora)] → Decollo (giro $giro di $GIRI)"
        if "$SCRIPT_DIR/takeoff.sh"; then
            break
        fi
        # Ripetere il decollo con il drone gia in aria lo fa rifiutare: se
        # sta salendo, il giro e andato a buon fine anche se lo script dice no.
        ALT="$(leggi_topic /mavros/global_position/rel_alt data 10)"
        if [ -n "$ALT" ] && awk "BEGIN{exit !($ALT > 5.0)}"; then
            echo "  il drone e comunque in volo a ${ALT} m: proseguo"
            break
        fi
        if [ "$giro" = "$GIRI" ]; then
            echo "   ERRORE: decollo fallito $GIRI volte. Motivo nel pannello 'mavproxy'." >&2
            exit 1
        fi
        echo "   riprovo fra 30 s"
        sleep 30
    done

    # Un blocco di sicurezza scattato durante il decollo lascia il drone in
    # RTL e la missione non ripartirebbe: va detto, non scavalcato.
    SICUREZZA="$(leggi_topic /sicurezza/stato data 10)"
    if [ -n "$SICUREZZA" ] && [ "$SICUREZZA" != "entro i limiti" ]; then
        echo "   ERRORE: blocco di sicurezza attivo: $SICUREZZA" >&2
        echo "   Riavvia lo stack (docker compose restart) e ripeti demo.sh." >&2
        exit 1
    fi

    echo "[$(ora)] → Avvio la missione"
    ros2 topic pub --once /mission/avvia std_msgs/msg/Bool "data: true" >/dev/null
    echo "  fase: $(leggi_topic /mission/stato data 10)"
fi

if [ "$VIDEO_S" != 0 ]; then
    echo "[$(ora)] → Registro ${VIDEO_S} s simulati"
    python3 "$SCRIPT_DIR/registra_video.py" "$VIDEO_S"
fi

echo
echo "[$(ora)] Demo in corso. Per seguirla:"
echo "  tmux attach -t drone          poi Ctrl-b n fino al pannello 'monitor'"
echo "  ros2 run rqt_image_view rqt_image_view /target/debug_image   (serve la grafica)"
