#!/usr/bin/env bash
#
# Esegue una prova completa e ne stampa il riepilogo.
#
#   prova.sh [etichetta] [durata_simulata]     default: senza etichetta, 60 s
#
# La durata e in secondi SIMULATI, non di orologio. Su una macchina senza
# accelerazione grafica la simulazione gira a una frazione del tempo reale
# (misurato 0.23x), quindi 60 s simulati possono richiedere quattro minuti di
# attesa: e il prezzo per avere prove della stessa lunghezza su macchine e
# carichi diversi, cioe confrontabili.
#
# Fa in un comando la sequenza che finora si eseguiva a mano: fotografa la
# configurazione, decolla se serve, avvia la missione, attende, e riassume il
# CSV prodotto. Il motivo per cui esiste e la ripetibilita: una procedura
# digitata a mano cambia ogni volta di qualche dettaglio, e quel dettaglio
# finisce nei numeri.
#
# Per confrontare due configurazioni:
#   prova.sh baseline 150
#   ...si modifica un parametro con ros2 param set...
#   prova.sh modifica 150
#   metriche.py confronta /ws/metrics/<baseline>.csv /ws/metrics/<modifica>.csv
#
set -e

ETICHETTA="${1:-}"
DURATA="${2:-60}"
CARTELLA="${CARTELLA_METRICHE:-/ws/metrics}"

source /opt/ros/jazzy/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash

echo "→ Controllo che lo stack sia in piedi"
NODI="$(ros2 node list 2>/dev/null || true)"
for atteso in detector_node tracker_node controller_node mission_node metrics_node; do
    if ! echo "$NODI" | grep -q "$atteso"; then
        echo "   ERRORE: /$atteso non risponde. Lo stack e avviato?" >&2
        echo "   docker compose exec sim start_all.sh --detach" >&2
        exit 1
    fi
done
echo "  tutti i nodi presenti."

# Con use_sim_time attivo i timer dei nodi non partono finche Gazebo non
# pubblica /clock: e la causa piu probabile di una prova che "non fa niente".
if ! ros2 topic echo /clock --once >/dev/null 2>&1; then
    echo "   ATTENZIONE: /clock non pubblica. Con use_sim_time i timer dei nodi" >&2
    echo "   restano fermi: verifica che Gazebo sia in esecuzione." >&2
fi

echo "→ Fotografo la configurazione attiva"
salva_config.sh "$ETICHETTA" >/dev/null
echo "  fatto."

# L'etichetta va passata a metrics_node prima dell'avvio della missione: il
# nodo apre un file nuovo quando la missione parte, e a quel punto rilegge il
# parametro. Cosi CSV e file dei parametri portano lo stesso nome.
if [ -n "$ETICHETTA" ]; then
    ros2 param set /metrics_node etichetta_config "$ETICHETTA" >/dev/null
fi

ALT="$(ros2 topic echo /mavros/global_position/rel_alt --field data --once 2>/dev/null | head -1 || true)"
if [ -z "$ALT" ] || awk "BEGIN{exit !(${ALT:-0} < 1.0)}"; then
    echo "→ Il drone e a terra: decollo"
    takeoff.sh
else
    echo "→ Il drone e gia in volo a ${ALT} m: salto il decollo"
fi

echo "→ Avvio missione"
ros2 topic pub --once /mission/avvia std_msgs/msg/Bool "data: true" >/dev/null
sleep 2

# Il file della prova in corso: e quello che metrics_node ha aperto all'avvio
# della missione, e porta l'etichetta se ne e stata data una.
file_prova() {
    if [ -n "$ETICHETTA" ]; then
        ls -t "$CARTELLA"/*"$ETICHETTA"*.csv 2>/dev/null | head -1
    else
        ls -t "$CARTELLA"/*.csv 2>/dev/null | head -1
    fi
}

# Il tempo di simulazione si legge dalla prima colonna del CSV. Prima si
# chiedeva a `ros2 topic echo /clock`, che pero intercala nell'uscita l'avviso
# "A message was lost!!!" quando la coda si riempie: quel testo finiva
# nell'aritmetica della shell, che moriva con un errore di sintassi chiudendo la
# prova dopo pochi secondi. Il CSV riporta lo stesso orologio — i nodi girano
# con use_sim_time — senza intermediari e senza costo.
ora_sim() {
    local f
    f="$(file_prova)"
    if [ -n "$f" ]; then
        tail -1 "$f" | cut -d, -f1 | cut -d. -f1
    fi
}

echo "→ Prova in corso per ${DURATA} s simulati"
SIM0="$(ora_sim)"
WALL0="$(date +%s)"
case "$SIM0" in
    ''|*[!0-9]*)
        echo "   ERRORE: non riesco a leggere il tempo di simulazione dal CSV." >&2
        echo "   metrics_node sta scrivendo in $CARTELLA? Gazebo pubblica /clock?" >&2
        exit 1
        ;;
esac

# Limite di sicurezza sull'orologio: se la simulazione si ferma del tutto, il
# tempo simulato non avanza mai e senza questo la prova non terminerebbe.
LIMITE_REALE=$(( DURATA * 20 + 120 ))

while true; do
    sleep 10
    SIM="$(ora_sim)"
    ORLOGIO="$(( $(date +%s) - WALL0 ))"
    case "$SIM" in
        ''|*[!0-9]*) SIM="" ;;
    esac
    if [ -z "$SIM" ]; then
        echo "  ${ORLOGIO}s reali  tempo simulato non leggibile"
    else
        TRASCORSO="$(( SIM - SIM0 ))"
        FASE="$(ros2 topic echo /mission/stato --field data --once 2>/dev/null \
                | grep -m1 -oE '(ATTESA|PATTUGLIAMENTO[^ ]*|AGGANCIO|RICERCA)' || echo '?')"
        # La distanza dal bersaglio e la quinta colonna da destra del CSV.
        DIST="$(tail -1 "$(file_prova)" | awk -F, '{print $23}')"
        echo "  ${TRASCORSO}s sim (${ORLOGIO}s reali)  fase=${FASE}  distanza=${DIST:-?} m"
        [ "$TRASCORSO" -ge "$DURATA" ] && break
    fi
    if [ "$ORLOGIO" -ge "$LIMITE_REALE" ]; then
        echo "   ATTENZIONE: ${LIMITE_REALE}s reali trascorsi senza raggiungere" >&2
        echo "   ${DURATA}s simulati. La simulazione e ferma o molto lenta." >&2
        break
    fi
done

echo
echo "→ Prova conclusa. Riepilogo:"
ULTIMO="$(file_prova)"
if [ -z "$ULTIMO" ]; then
    echo "   Nessun CSV in $CARTELLA: metrics_node ha scritto altrove?" >&2
    exit 1
fi
metriche.py riassumi "$ULTIMO"

echo
echo "File della prova: $ULTIMO"
echo "La missione resta in corso: per fermare il drone"
echo "  ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \"{base_mode: 0, custom_mode: 'LOITER'}\""
