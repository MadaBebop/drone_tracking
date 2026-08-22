# Come contribuire

Grazie dell'interesse. Questo è un progetto universitario a singolo autore, quindi
i contributi sono benvenuti ma le decisioni architetturali restano al responsabile.
Prima di scrivere codice, apri una issue: si risparmia tempo a entrambi.

## Prepararsi

Il modo più rapido per avere un ambiente funzionante è il container, che compila
ArduPilot e i plugin Gazebo da sorgente:

```bash
docker compose build && docker compose up -d && docker compose exec sim start_all.sh
```

Per l'installazione nativa su Ubuntu 24.04 la procedura completa è nel
[README](README.md), sezione *Avvio manuale*.

## Tre trappole che fanno perdere un pomeriggio

Sono documentate nel README ma vale la pena ripeterle, perché ci siamo già cascati.

**Gli asset Gazebo esistono in due copie.** Il repository versiona mondo e modello
sotto `sim/`, ma l'avvio nativo li carica da `ardupilot_gazebo/`. Un `git pull`
aggiorna i primi mentre Gazebo legge i secondi, e le modifiche sembrano non avere
effetto. La soluzione è il symlink descritto nel README.

**I topic MAVROS pubblicano in BEST_EFFORT.** `ros2 topic hz` e `ros2 topic echo`
usano QoS RELIABLE e non ricevono nulla, facendo credere che il topic sia muto.
Per verificarli serve un subscriber best-effort.

**I nomi dei parametri ArduPilot cambiano fra versioni.** Un parametro inesistente
viene ignorato in silenzio: si crede di averlo impostato e invece no. Da Copter 4.7
diversi nomi sono passati a unità SI (`WPNAV_ACCEL` → `WP_ACC`) e `ARMING_CHECK`
è diventato `ARMING_SKIPCHK`, con logica invertita. Verifica sempre con
`param show`.

## Modificare i nodi

Dopo ogni modifica ai file Python serve ricompilare il workspace, altrimenti ROS
continua a usare la versione precedente:

```bash
colcon build --packages-select drone_tracking && source install/setup.bash
```

Nel container i nodi sono montati come volume con `--symlink-install`: basta
riavviare il pannello `nodes`.

## Convenzioni di codice

- **Commenti in italiano**, come il resto del progetto. Spiega il *perché*, non il
  *cosa*: il codice dice già cosa fa.
- **Niente costanti in unità arbitrarie.** Tempi in secondi, distanze in metri,
  velocità in m/s. Contare messaggi ricevuti per misurare il tempo è stata la
  causa di diversi bug: i topic hanno frequenze diverse fra loro e variabili nel
  tempo.
- **Il campo `z` di `Point` non è una coordinata.** Trasporta l'area del contorno
  e funge da indicatore di validità. Va preservato.
- **Controlla gli esiti.** `subprocess.run` senza verifica del codice di uscita e
  chiamate a servizi senza controllo della risposta hanno prodotto guasti
  silenziosi difficili da diagnosticare.

## Verificare le modifiche

Il progetto non ha una suite di test automatici: si verifica misurando in
simulazione. Non tarare a occhio.

```bash
monitor.sh            # cruscotto live: fase, quota, catena di percezione
snapshot.sh 30        # salva i frame annotati come PNG in snapshots/
```

Per una modifica che tocca il controllo o il filtro, riporta nella pull request
almeno: tempo trascorso in `AGGANCIO`, distanza mediana dal bersaglio, e frazione
di fotogrammi in cui il bersaglio è inquadrato. Gli script sopra e gli esempi nel
README mostrano come ricavarli.

## Pull request

- Un argomento per pull request.
- Messaggio di commit che spiega il problema e la sua causa, non solo la modifica.
  I commit esistenti sono il modello.
- Se hai misurato un miglioramento, mettine i numeri nel messaggio.
- Aggiorna il README se cambi comportamento, parametri o procedura di avvio.

## Segnalare problemi

Usa i modelli di issue. Per i malfunzionamenti serve sapere in quale ambiente
girava — container o installazione nativa — perché i due divergono facilmente.

Per le vulnerabilità non aprire una issue pubblica: vedi [SECURITY.md](SECURITY.md).

## Licenza

Contribuendo accetti che il tuo contributo sia distribuito con licenza MIT, come
il resto del progetto.
