# Drone Tracking con Visione Artificiale e Resistenza al Jamming
### Progetto universitario — ROS 2 Jazzy + ArduPilot SITL + Gazebo Harmonic

---

## Panoramica

Sistema di controllo autonomo per drone capace di:

- Seguire un **percorso di pattugliamento** a waypoint predefiniti
- **Rilevare e agganciare un bersaglio mobile** tramite visione artificiale a colore
- **Mantenere il tracking** sotto jamming GPS/RF grazie a un filtro di Kalman con
  incertezza di misura adattiva
- **Ricercare il bersaglio** con una spirale espandibile quando lo perde di vista
- Confrontarsi con un bersaglio **non collaborativo**, che tenta l'evasione una
  volta accortosi di essere inseguito

Il progetto nasce dall'analisi delle tecnologie drone impiegate nel conflitto
russo-ucraino, dove il jamming GPS di massa ha reso necessari sistemi di
navigazione autonomi basati su visione artificiale.

---

## Architettura del sistema

```
Gazebo Harmonic (simulazione 3D fisica)
        ↕ JSON UDP (porte 9012/9013)
ArduPilot SITL (firmware Copter 4.7.0)
        ↕ MAVLink TCP (porta 5760)
MAVProxy (hub MAVLink / ground control station)
        ↕ UDP (porta 14555)
MAVROS2 (bridge MAVLink ↔ ROS 2)
        ↕ ROS 2 topics/services
┌───────────────────────────────────────────────┐
│                  Nodi ROS 2                   │
│                                               │
│  detector_node                                │
│    → segmentazione HSV del bersaglio rosso    │
│         ↓ /target/position                    │
│  jammer_node                                  │
│    → corrompe il segnale, simula EW           │
│         ↓ /target/jammed_position             │
│         ↓ /rf/noise_level                     │
│  tracker_node                                 │
│    → Kalman con R adattiva al rumore RF       │
│         ↓ /target/tracked_position            │
│  ├─ controller_node                           │
│  │    → PD su velocità in body frame          │
│  │      ↓ /mavros/setpoint_velocity/…         │
│  └─ mission_node                              │
│       → macchina a stati, waypoint, ricerca   │
│         ↓ /mavros/setpoint_position/local     │
│         ↓ /mission/stato                      │
│  target_mover_node                            │
│    → muove ed evade il bersaglio in Gazebo    │
│  metrics_node                                 │
│    → registra la prova su CSV, con la verità  │
│      a terra letta da Gazebo                  │
└───────────────────────────────────────────────┘
```

Il topic `/mission/stato` è il canale di sincronizzazione: detector, tracker,
controller e target_mover cambiano tutti comportamento in base alla fase corrente.

---

## Stack tecnologico

| Componente | Versione |
|---|---|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy Jalisco |
| Simulatore | Gazebo Harmonic (gz-sim8) |
| Firmware | ArduPilot SITL — Copter 4.7.0 (container) / `master` 4.8.0-dev (sviluppo) |
| Bridge | MAVROS2 + `ros_gz_bridge` |
| Visione | OpenCV (segmentazione HSV) |
| Navigazione | Filtro di Kalman custom `[x, y, vx, vy]` |

---

## Avvio rapido con Docker

Il modo più semplice per far girare tutto senza installare nulla. Serve solo
Docker con Compose.

**Prerequisito** — copiare gli asset Gazebo personalizzati (mondo con il
bersaglio e drone con la telecamera) in `sim/`. ArduPilot e `ardupilot_gazebo`
**non** servono: il `Dockerfile` li compila da sorgente. Vedi
[sim/README.md](sim/README.md).

```bash
docker compose build
```

La prima build dura 20–40 minuti (compila ArduPilot da zero); le successive
sfruttano la cache.

```bash
docker compose up -d
docker compose exec sim start_all.sh
```

`start_all.sh` apre una sessione tmux con un pannello per componente — Gazebo,
SITL, MAVProxy, MAVROS, nodi ROS 2 — avviati in sequenza con i ritardi giusti.
Dopo ~30 secondi, dal pannello `shell`:

```bash
takeoff.sh
```

```bash
ros2 topic pub --once /mission/avvia std_msgs/msg/Bool "data: true"
```

Comandi tmux utili: `Ctrl-b n`/`Ctrl-b p` per cambiare pannello, `Ctrl-b d` per
staccarsi lasciando tutto in esecuzione.

```bash
docker compose exec sim tmux kill-session -t drone
```

### Prova guidata

Procedura per verificare che l'intera catena funzioni, con cosa aspettarsi a ogni
passo. Tutti i comandi vanno dati dal pannello `shell` di tmux, oppure con
`docker compose exec sim <comando>` da un altro terminale.

**1. Verifica che lo stack sia su**

```bash
ros2 node list | grep -cE '(detector|tracker|jammer|controller|mission|target_mover)_node'
```

Deve rispondere **6**, uno per nodo del progetto. Se è meno, qualche pannello non
è partito: controlla con `tmux attach -t drone`. Nota che `ros_gz_bridge` e
`mavros_node` compaiono nell'elenco ma non in questo conteggio, e MAVROS impiega
qualche secondo in più degli altri a registrarsi.

**2. Stato iniziale**

```bash
ros2 topic echo /mission/stato --once
```

Deve dire `ATTESA`. In questa fase il detector **non** pubblica: se controlli
`/target/position` troverai `(0, 0, 0)`. È il comportamento corretto, non un
guasto.

**3. Decollo**

```bash
takeoff.sh
```

Poi verifica che salga davvero:

```bash
ros2 topic echo /mavros/global_position/rel_alt --once
```

Attendi che il valore sia stabile intorno a 12. Se resta a zero, guarda il
pannello `mavproxy`: quasi sempre è un prearm check che ha rifiutato l'arming.

**4. Avvio della missione**

```bash
ros2 topic pub --once /mission/avvia std_msgs/msg/Bool "data: true"
```

Da qui lo stato passa a `PATTUGLIAMENTO:0` e il numero cresce man mano che i
waypoint vengono raggiunti. Il rilevamento si attiva 2 secondi dopo.

**5. Monitora la missione**

Il modo più comodo è il cruscotto live, che raccoglie tutto su una schermata:

```bash
monitor.sh
```

```
  Fase        AGGANCIO
  Quota        11.8 m        Posizione   x  17.1  y  21.9
  Camera      11.2 Hz
  Datalink    JAMMING ATTIVO  rumore RF 0.8

  Catena di percezione  (posizione orizzontale nel frame)
  detector    +0.209  ····················|···#················
  jammer       assente  ·········································
  kalman       assente  ·········································

  Comando     laterale -0.78 m/s   longitudinale +0.53 m/s
```

Le tre barre sono lo stesso bersaglio nei tre stadi della catena: `#` è la sua
posizione orizzontale nel frame, `|` il centro. Lo scatto qui sopra è preso
durante una finestra di jamming, nell'istante in cui il jammer ha scartato il
pacchetto: il detector vede il bersaglio, gli stadi a valle no.

`start_all.sh` lo avvia già nel pannello tmux `monitor`. Per lanciarlo a mano con
un intervallo diverso: `monitor.sh 1.0`.

**5b. Osserva i singoli topic**

Se vuoi i numeri grezzi invece del cruscotto, gli stessi dati nei tre stadi.

```bash
ros2 topic echo /target/position --field x
```

```bash
ros2 topic echo /target/jammed_position --field x
```

```bash
ros2 topic echo /target/tracked_position --field x
```

Durante le finestre di jamming (~4 s ogni ~14 s) il secondo valore deve risultare
visibilmente più rumoroso del primo, e ogni tanto azzerarsi di colpo — è la
perdita di pacchetto simulata. Il terzo deve restare molto più liscio di
entrambi: è la R adattiva che fa scendere il guadagno di Kalman.

Per sapere quando il jammer è attivo:

```bash
ros2 topic echo /gps/jammed
```

**6. Guarda cosa vede il drone**

Senza GUI, i frame annotati si salvano come PNG:

```bash
snapshot.sh 30
```

I file finiscono in `./snapshots/` sull'host, apribili da Esplora Risorse. Sul
frame trovi il contorno del bersaglio in verde, il centroide in giallo e le
coordinate normalizzate con l'area. Per il feed grezzo senza annotazioni:

```bash
snapshot.sh 10 /drone/camera/image_raw
```

**7. Aggancio ed evasione**

Quando il drone arriva sul waypoint `(20, 20)` e conferma il bersaglio per 5
frame, lo stato passa ad `AGGANCIO`. Cinque secondi dopo il bersaglio inizia la
fuga, e se il drone lo perde per ~2 s lo stato diventa `RICERCA` con la spirale.
Per seguire tutte le transizioni:

```bash
ros2 topic echo /mission/stato
```

### GUI di Gazebo

Di default il container gira **headless** (`HEADLESS=1`): parte solo il server
fisico, senza finestra. È la modalità consigliata — la simulazione è più veloce
e non serve alcuna configurazione grafica.

Per vedere la finestra 3D servono `HEADLESS=0` e un server X raggiungibile:

- **Linux** — decommentare `DISPLAY`, `QT_X11_NO_MITSHM` e il volume
  `/tmp/.X11-unix` in [docker-compose.yml](docker-compose.yml), poi
  `xhost +local:docker`
- **Windows 11** — funziona via WSLg lanciando Docker dentro WSL2, con
  `DISPLAY=$DISPLAY` e il volume `/tmp/.X11-unix`
- **macOS** — serve XQuartz con "Allow connections from network clients"

Per il solo tracking la GUI non serve: `rqt_image_view` sul topic
`/target/debug_image` mostra già cosa vede il drone.

### Collegare una ground control station esterna

Le porte `5760/tcp` (MAVLink diretto dal SITL) e `14550/udp` (uscita MAVProxy)
sono esposte sull'host: Mission Planner o QGroundControl possono collegarsi
dall'esterno del container.

### Modificare i nodi senza ricostruire

`src/drone_tracking/drone_tracking` e `launch/` sono montati come volume e
l'immagine è costruita con `--symlink-install`: le modifiche ai file Python sono
visibili subito, basta riavviare il pannello `nodes`. Serve un `docker compose
build` solo se cambiano `setup.py`, `package.xml` o gli asset in `sim/`.

---

## Struttura del progetto

```
drone_tracking/
├── docker/
│   ├── Dockerfile              # immagine all-in-one (build multi-stage)
│   ├── entrypoint.sh           # source degli ambienti ROS 2
│   └── start_all.sh            # orchestrazione tmux dello stack
├── docker-compose.yml
├── scripts/
│   ├── takeoff.sh              # arm + GUIDED + decollo via servizi MAVROS
│   └── export_sim_assets.sh    # estrae i file Gazebo custom dalla VM
├── sim/                        # asset Gazebo personalizzati (vedi sim/README.md)
│   ├── worlds/iris_runway.sdf
│   └── models/iris_with_ardupilot/
└── src/
    └── drone_tracking/
        ├── drone_tracking/
        │   ├── detector_node.py       # rilevamento bersaglio (OpenCV)
        │   ├── tracker_node.py        # filtro di Kalman
        │   ├── jammer_node.py         # simulazione guerra elettronica
        │   ├── controller_node.py     # controllo PD del drone
        │   ├── mission_node.py        # macchina a stati della missione
        │   └── target_mover_node.py   # movimento ed evasione del bersaglio
        ├── launch/
        │   └── tracking.launch.py     # 6 nodi + ponte ros_gz per la camera
        ├── package.xml
        └── setup.py
```

---

## Nodi ROS 2

### mission_node

Macchina a stati che governa l'intera missione. La fase corrente è pubblicata su
`/mission/stato` e condiziona il comportamento di tutti gli altri nodi.

| Fase | Comportamento |
|---|---|
| `ATTESA` | Drone a terra. Il detector non rileva, il tracker resta azzerato. |
| `PATTUGLIAMENTO` | Percorre i waypoint pubblicando su `/mavros/setpoint_position/local`. Il rilevamento si attiva 2 s dopo l'avvio, per dare tempo al decollo. |
| `AGGANCIO` | Bersaglio confermato: il controllo passa a `controller_node`. |
| `RICERCA` | Bersaglio perso: spirale espandibile attorno all'ultima posizione nota. |

**Percorso di pattugliamento** — circuito quadrato a 12 m di quota, con soglia
di raggiungimento pari al parametro `soglia_waypoint`:

```
(0,0) → (20,0) → (20,20) → (0,20) → (0,0)
```

Il vertice `(20,20)` coincide con la zona in cui orbita il bersaglio. Se il giro
si chiude senza aggancio, il pattugliamento riparte dal waypoint 1.

**Aggancio** — richiede `frame_conferma_richiesti` messaggi consecutivi con
bersaglio visibile, e solo sopra i 2 m di quota, per evitare falsi positivi
durante il decollo.

**Perdita e ricerca** — trascorso `soglia_avvia_ricerca_s` senza bersaglio in
`AGGANCIO`, la fase passa a `RICERCA`: il drone descrive una spirale attorno
alla propria posizione al momento della perdita, con raggio iniziale 3 m che
cresce di `ricerca_vel_espansione` al secondo fino a `ricerca_raggio_max`, oltre
il quale la ricerca è dichiarata fallita e la missione torna a
`PATTUGLIAMENTO`. Bastano `frame_conferma_riaggancio` messaggi consecutivi di
nuova visibilità per tornare in `AGGANCIO`.

Il conteggio della perdita avviene sia all'arrivo dei messaggi del tracker sia
nel timer periodico: se `/target/tracked_position` tace del tutto — detector
fermo, ponte delle immagini caduto — nessun messaggio farebbe partire il
conteggio, e la fase resterebbe `AGGANCIO` a tempo indeterminato.

### detector_node

Legge il feed della telecamera montata sul drone (puntata a **nadir**, FOV
orizzontale 90°, 640×480, `update_rate` dichiarato 30 Hz ma reso molto piu
basso dal rasterizzatore software — vedi *Problemi noti*) e isola il bersaglio
rosso con doppia soglia HSV: due intervalli, perché il rosso è a cavallo del
wrap-around della tinta.

```
mask1: H ∈ [0, tolleranza_tinta]         S ≥ saturazione_minima  V ≥ valore_minimo
mask2: H ∈ [180 − tolleranza_tinta, 180] S ≥ saturazione_minima  V ≥ valore_minimo
```

I tre estremi sono parametri ROS (`tolleranza_tinta`, `saturazione_minima`,
`valore_minimo`): saturazione e valore minimi escludono i grigi e le zone in
ombra, che rientrerebbero nella tinta giusta.

Del contorno più grande calcola il centroide via momenti di immagine e lo
normalizza in `[-1, +1]`. I contorni sotto `area_minima_px` vengono scartati
come rumore visivo.

Il campo `z` del messaggio trasporta l'**area del contorno** e funge da flag di
visibilità: `z > 0` significa bersaglio presente. Tutti i nodi a valle usano
questa convenzione — è ciò che distingue "bersaglio al centro dell'immagine" da
"bersaglio assente", due casi che le sole coordinate `(0, 0)` confonderebbero.

> **Nota sullo smoothing.** È presente un filtro esponenziale con `alpha`
> configurabile, ma il valore è impostato a `1.0` (nessun filtraggio): il lag
> introdotto faceva perdere l'aggancio durante l'evasione del bersaglio ad alta
> velocità. Il codice resta per poter riattivare lo smoothing con bersagli lenti.

Pubblica anche `/target/debug_image` con contorno, centroide e coordinate
disegnati sul frame.

### jammer_node

Simula un sistema di guerra elettronica che si interpone tra detector e tracker.
Cicla automaticamente ON/OFF su un timer a 10 Hz: `jam_on_duration` tick attivo,
`jam_off_duration` inattivo.

Quando il jamming è attivo:

- pubblica `/gps/jammed: true` e `/rf/noise_level: 0.8`
- inietta rumore gaussiano di deviazione standard `deviazione_rumore` sulle
  coordinate, con clamp a `[-1, +1]`
- con probabilità `probabilita_perdita_segnale` simula la perdita totale del
  pacchetto (azzera x, y, z)

**Il disturbo è ripetibile.** Il generatore pseudocasuale parte da un seme
fisso, il parametro `seed`: due prove della stessa configurazione ricevono lo
stesso disturbo, quindi il loro confronto misura la modifica al codice e non due
sequenze di rumore diverse. Con `seed` negativo si torna al comportamento non
deterministico.

Due accorgimenti importanti: non corrompe segnali già nulli (non genera falsi
positivi dal nulla) ed è limitato a un messaggio ogni 50 ms, perché a piena
frequenza il tracker veniva sovraccaricato.

### tracker_node

Filtro di Kalman a 4 stati `[x, y, vx, vy]` in coordinate immagine.

| Matrice | Nome nel codice | Ruolo |
|---|---|---|
| F | `evoluzione_stato` | Modello cinematico a velocità costante, con `dt` reale |
| H | `mappa_osservazione` | Osserva solo posizione x/y |
| Q | `_matrice_Q(dt)` | Rumore di processo, ricostruito sul `dt` effettivo |
| R | `incertezza_sensore` | Adattiva, vedi sotto |
| K | `guadagno_kalman` | Bilancia modello e misura |

**Q dipende dal `dt`.** Era una matrice costante, sommata identica a ogni
predizione qualunque fosse il tempo trascorso: al ritmo variabile della
telecamera lo stesso intervallo veniva penalizzato o premiato a caso. Ora si usa
la discretizzazione standard di un modello a velocità quasi costante: da
un'accelerazione ignota di intensità `intensita_rumore_accel`, integrata su
`dt`, seguono una varianza `q·dt³/3` sulla posizione, `q·dt` sulla velocità e
una covarianza `q·dt²/2` fra le due — termini incrociati che nel modello
esistono e che la matrice diagonale precedente ignorava.

**R adattiva** — è il punto centrale della resistenza al jamming. Il tracker si
iscrive a `/rf/noise_level` e ricalcola l'incertezza di misura a ogni variazione:

```
R = rumore_sensore_base + (rumore_sensore_max − rumore_sensore_base) · livello_rumore
```

Con il jamming attivo (`livello_rumore = 0.8`) l'incertezza di misura sale di
circa 30 volte: il guadagno di Kalman crolla e il filtro dà fiducia alla propria
predizione invece che ai dati corrotti. Senza jamming torna a fidarsi del
detector. È l'equivalente in coordinate immagine di un INS che degrada
gracefully quando il GNSS diventa inaffidabile.

**Gestione della perdita di segnale** — durante i buchi il filtro pubblica la
predizione, tollerando fino a `soglia_perdita` messaggi consecutivi; oltre
quella soglia si resetta e pubblica un punto nullo, segnalando la perdita a
mission e controller.

**Sullo smorzamento della velocità stimata, rimosso.** Una versione precedente
moltiplicava per 0.6 la velocità nello stato a ogni correzione, per contenere le
predizioni sbagliate. Era uno smorzamento applicato allo stato senza toccare la
covarianza corrispondente: il filtro dichiarava una fiducia che non
corrispondeva più alla stima, e la coerenza fra le due è l'unica cosa che rende
ottimo un filtro di Kalman. Peggio, il fattore si componeva ad ogni misura:
dopo dieci aggiornamenti la velocità stimata era lo 0.6% di quella calcolata, e
la predizione durante una perdita di segnale restava ferma sull'ultima posizione
invece di estrapolare il moto del bersaglio. Lo stesso effetto — una stima di
velocità meno nervosa — si ottiene ora per la via corretta, cioè scegliendo
`intensita_rumore_accel`, che governa quanto la velocità può cambiare fra due
misure aggiornando di conseguenza anche l'incertezza.

### controller_node

Controllo **proporzionale-derivativo** che traduce l'errore di posizione nel
frame della telecamera in comandi di velocità in body frame.

Tutti i valori sono parametri ROS: la colonna riporta il default nel codice,
non un numero fisso. `ros2 param get /controller_node kp_x` dice quale era
attivo durante una prova.

| Parametro | Default | Unità |
|---|---|---|
| `kp_x`, `kp_y` | 1.2 | 1/s |
| `kd_x`, `kd_y` | 0.35 | — |
| `vel_max` | 5.0 | m/s (per asse) |
| `deadzone` | 1.0 | metri |
| `durata_coasting_s` | 2.0 | s |
| `timeout_percezione_s` | 0.5 | s |
| `timeout_posa_s` / `timeout_quota_s` | 1.0 / 2.0 | s |
| Frequenza di pubblicazione | 10 | Hz |

**I guadagni agiscono su metri, non su pixel.** L'errore normalizzato d'immagine
viene prima convertito in scostamento al suolo:

```python
error = norm * quota * tan(semi_fov)
```

Le coordinate normalizzate cambiano significato al variare di quota e ottica —
lo stesso `0.3` vale 2 m a 12 m con FOV 60° e 3.6 m con FOV 90° — quindi una
taratura fatta su di esse va rifatta a ogni modifica. In metri il guadagno ha un
senso fisico diretto: `kp = 1.2` significa 1.2 m/s di comando per ogni metro di
scarto, cioè uno scarto a regime di `velocità_bersaglio / kp` ≈ 1 m contro un
bersaglio a 1.2 m/s. La quota entra nella conversione, quindi non serve più
scalare i guadagni separatamente.

**Mappatura degli assi** — l'errore in pixel normalizzati diventa velocità nel
frame del drone:

- `error_x` → velocità laterale, segno invertito
- `error_y` → velocità longitudinale, segno invertito

**Rotazione nel frame del mondo** — il comando così ottenuto **non** va
pubblicato direttamente. `/mavros/setpoint_velocity/cmd_vel_unstamped` viene
tradotto da MAVROS in `SET_POSITION_TARGET_LOCAL_NED` con frame `LOCAL_NED`,
cioè il frame del **mondo**, non quello del velivolo. Il controller ruota quindi
il vettore con lo yaw letto da `/mavros/local_position/pose`:

```python
cmd.linear.x = v_avanti * cos(yaw) - v_laterale * sin(yaw)
cmd.linear.y = v_avanti * sin(yaw) + v_laterale * cos(yaw)
```

Senza questa rotazione il sistema è corretto solo con yaw esattamente zero. In
volo lo yaw non è comandato e deriva: misurato fra 25° e 47°, con oscillazioni di
±20°. Il comando finiva **in media 61° fuori bersaglio** e il drone spingeva di
traverso, incapace di seguire perfino l'orbita lenta del bersaglio.

| | Prima | Dopo |
|---|---|---|
| Scarto medio comando/bersaglio | −61.3° | **−8.6°** |
| Scarto mediano | −61.3° | −11.4° |
| Tempo in `AGGANCIO` | — | 98.3% |
| Distanza minima raggiunta | — | 2.9 m |

Per diagnosticare guasti di questo tipo non basta osservare se il drone si muove:
va confrontata la **direzione** del comando con la direzione reale del bersaglio.
Uno scarto sistematico costante indica un frame sbagliato, non una taratura da
rivedere.

**Compensazione d'assetto** — è la correzione che ha reso possibile
l'inseguimento. La telecamera è solidale al corpo, quindi una rotazione del
velivolo trasla l'immagine **indipendentemente da dove sia il bersaglio**. Con
FOV orizzontale 90° su 640×480 i semicampi valgono 0.785 rad in orizzontale e
0.644 in verticale: bastano pochi gradi di pitch per spostare il bersaglio di
una frazione visibile del campo. Il controller sottrae quindi il contributo
dell'assetto, lavorando **sugli angoli** e non sulle coordinate normalizzate:

```python
alpha_x = atan(msg.x * tan(semi_fov_o)) - roll
alpha_y = atan(msg.y * tan(semi_fov_v)) + pitch
norm_x  = tan(alpha_x) / tan(semi_fov_o)
norm_y  = tan(alpha_y) / tan(semi_fov_v)
```

In una proiezione prospettica vale `u = tan(alpha)/tan(semi_fov)`, quindi
coordinata e angolo non sono proporzionali. Una versione precedente divideva
l'assetto per il semicampo **in radianti**, che è il primo termine dello sviluppo
della tangente: sovracorreggeva del 27% in orizzontale e del 16% in verticale,
cioè a 25° di rollio introduceva un errore fantasma di circa 1.3 m al suolo a
12 m di quota.

Senza questa sottrazione l'errore misurava l'inclinazione del drone più della
posizione del bersaglio: correlazione **r = −0.665** fra pitch ed errore
verticale, con l'errore che spazzava l'intero campo visivo (−0.98…+0.97) mentre
il bersaglio era pressoché fermo. Il risultato era una retroazione che divergeva
in pochi secondi — il drone agganciava, inseguiva un paio di secondi e perdeva.

| | Senza compensazione | Con compensazione |
|---|---|---|
| Distanza mediana dal bersaglio | 10.7 m | **3.0 m** |
| Tempo entro 8 m | — | **98%** |
| Tempo in `AGGANCIO` | 98.3% | **100%** |
| Campioni con bersaglio visibile in 100 s | 219 | 892 |

**Campo visivo largo** — l'ottica è a 90° invece di 60°: a 12 m di quota
l'impronta a terra passa da ~14 a ~24 m. Con il campo stretto il bersaglio
usciva dall'inquadratura appena il drone si inclinava, e veniva perso dopo mezzo
secondo. Alzare i guadagni non aiutava, anzi: più il drone è aggressivo più si
inclina, e il bersaglio esce prima. Misurato — frazione di frame con bersaglio
inquadrato: **57%** con FOV 60° e guadagni alzati, **84%** con FOV 90° e guadagni
metrici.

**Guardia FOV** — le posizioni con `|x| > 1.2` o `|y| > 1.2` non vengono
inseguite: sono predizioni di Kalman ormai fuori dal campo visivo. Non azzerano
però il comando di colpo, perché è la situazione tipica di una fuga veloce (il
bersaglio scivola verso il bordo poco prima di sparire): si passa al coasting,
descritto sotto.

**Coasting alla perdita di vista** — quando il tracker rinuncia, il comando non
viene azzerato di netto ma smorzato a zero su `durata_coasting_s`, proseguendo
nella direzione in cui il bersaglio si stava muovendo, che è la più probabile
per riacquisirlo. Prima il drone restava immobile per tutta l'attesa che precede
`RICERCA`: la sequenza reale era ~1.4 s di inseguimento sulla predizione di
Kalman, poi comando nullo fino allo scadere di `soglia_avvia_ricerca_s`.

**Watchdog sugli ingressi** — il timer di pubblicazione verifica che percezione,
posa e quota si stiano ancora aggiornando; se una tace oltre la propria soglia,
il comando viene azzerato e l'evento loggato come errore. Senza questa verifica
la morte di `detector_node`, o un ponte immagini che si ferma, lasciava il drone
a ripetere all'infinito l'ultima velocità nota — volo alla cieca fino a
`vel_max`, senza che nulla nei log lo segnalasse. Il comando azzerato viene
comunque pubblicato: interrompere lo stream di setpoint farebbe uscire ArduPilot
dalla modalità GUIDED.

**Anti derivative-kick** — al primo frame dopo ogni aggancio la derivata è
azzerata, altrimenti il salto iniziale dell'errore produrrebbe uno strappo.

Un timer dedicato a 10 Hz ripubblica il comando corrente su
`/mavros/setpoint_velocity/cmd_vel_unstamped`: ArduPilot esce dal controllo in
velocità se non riceve setpoint con continuità. Pubblica solo in fase `AGGANCIO`
e sopra 1 m di quota.

### target_mover_node

Muove la sfera rossa in Gazebo comandandone la posa tramite il servizio
`set_pose`, con due comportamenti:

| Fase | Comportamento | Velocità |
|---|---|---|
| `PATTUGLIO` | Orbita circolare attorno a `(20, 20)`, raggio `raggio_orbita` | `velocita_angolare` (0.35 rad/s ≈ **1.05 m/s**) |
| `EVASIONE` | Fuga in linea retta opposta al drone, per `durata_evasione_s` | `vel_evasione` a regime, raggiunta con rampa `accel_evasione` |

**Perché queste velocità.** Il limite non è la velocità massima del drone, che
arriva a 8 m/s, ma l'**errore a regime** del controllo proporzionale: inseguendo
un bersaglio a velocità costante, l'errore d'immagine si stabilizza intorno a
`velocità_bersaglio / kp`. La tabella seguente è stata misurata con `kp = 4.0`,
valore di una taratura precedente su coordinate normalizzate (il default attuale
è 1.2 e agisce su metri):

| Velocità bersaglio | Errore a regime | Esito |
|---|---|---|
| 1.2 m/s | ~0.30 | margine ampio, regge anche sotto jamming |
| 2.0 m/s | ~0.50 | metà semicampo: la prima finestra di jamming lo fa uscire |
| 4.2 m/s | oltre il campo | mai agganciato — era il valore originale dell'orbita |

Misure su 100 s di missione, distanza orizzontale drone-bersaglio:

| | Fuga a 2.0 m/s | Fuga a 1.2 m/s |
|---|---|---|
| Distanza massima | 48.5 m | 26.2 m |
| Distanza finale | 43.4 m | **6.3 m** |
| Tempo in `AGGANCIO` | 76% | 68% |

Con la fuga a 2 m/s il drone terminava a 43 m e in allontanamento; a 1.2 m/s
recupera e si riporta sopra il bersaglio. Per rendere l'inseguimento più difficile
conviene alzare `kp_x`/`kp_y` in `controller_node` insieme alla velocità, non la
velocità da sola.

L'evasione scatta dopo che la missione è entrata in `AGGANCIO`: il bersaglio si
comporta come un veicolo che si accorge di essere inseguito e reagisce con un
ritardo. Terminata la fuga riprende a orbitare attorno alla nuova posizione — è
ciò che mette davvero alla prova il tracker e la fase di `RICERCA`.

Il ritardo è `ritardo_evasione_s = 10.0`, misurato sull'orologio e non contando
messaggi. Il conteggio riparte da capo se l'aggancio si interrompe: il bersaglio
fugge solo dopo essere stato inseguito per **10 secondi consecutivi**, quindi con
un tracking discontinuo il ritardo osservato è più lungo. Terminata la fuga,
lunga `durata_evasione_s`, riprende a orbitare attorno alla nuova posizione.

Il comportamento osservato in simulazione è quello descritto sopra: è questo nodo
a muovere il bersaglio. Il modello nel world dichiara anche un plugin
`TrajectoryFollower` che però resta inerte — vedi *Problemi noti*.

### metrics_node

Registra la prova su un file CSV, uno per esecuzione, in `/ws/metrics` (montato
sull'host come `./metrics/`). Campiona a frequenza fissa — indipendente
dall'arrivo dei messaggi, così il file si media e si diagramma senza
reinterpolare — e riporta per ogni istante: fase della missione, validità e
coordinate del rilevamento e della stima, posizione secondo l'EKF, posizione
**vera** di drone e bersaglio letta da Gazebo, distanza fra i due, stato del
jamming, ritmo effettivo della catena di percezione.

La verità a terra viene da `/world/iris_runway/pose/info` e non da MAVROS,
perché la stima dell'EKF è essa stessa oggetto di misura e non può fare da
riferimento a se stessa. Entrambe sono registrate: la colonna `dist_xy_ekf`
accanto a `dist_xy_gt` rende visibile nei dati lo scarto fra le due, che il resto
del progetto assume nullo.

Alla chiusura il nodo stampa un riepilogo (campioni, percentuale di fotogrammi
con bersaglio, distanza media, tempo per fase).

### gnss_denial_node

Attacca il ricevitore satellitare **del drone**, iniettando il disturbo nei
parametri del SITL invece di simularlo a livello di topic: quello che viene
messo alla prova è il sistema reale, autopilota compreso, non una sua
imitazione. È un nodo distinto da `jammer_node` perché i due guasti sono
fisicamente diversi — quello disturba il canale con cui il bersaglio viene
rilevato, questo il GPS del velivolo — e confonderli era il difetto principale
della prima versione, in cui `/gps/jammed` veniva pubblicato ma non guidava
nulla.

| Modo | Parametro | Effetto |
|---|---|---|
| `jamming` | `SIM_GPS1_JAM = 1` | fix intermittente: misurato `fix_type` da 6 a 1, satelliti da 10 a 3, accuratezza dichiarata fino a 191 m |
| `negazione` | `SIM_GPS1_ENABLE = 0` | ricevitore muto, equivalente a un'antenna staccata |
| `spoofing` | `SIM_GPS1_GLTCH_X/Y` | fix falsificato di un offset in gradi (0.0002° ≈ 22 m) |

Non parte con lo stack: va abilitato esplicitamente, altrimenti non esisterebbe
più una linea di riferimento con cui confrontarlo.

```bash
docker compose exec -e LAUNCH_ARGS="gnss_denial:=true gnss_modo:=negazione" sim start_all.sh --detach
```

Il nodo non tocca il GPS prima che la missione lasci `ATTESA`: arming e decollo
hanno bisogno di un fix valido, e negarlo li farebbe fallire per un motivo che
non ha niente a che vedere con l'esperimento. Le chiamate a `/mavros/param/set`
sono asincrone — una risposta lenta bloccherebbe la pubblicazione di
`/gps/denial_active`, falsando l'annotazione delle finestre di attacco nei dati
— e alla chiusura i parametri vengono ripristinati, perché un attacco rimasto
impostato farebbe partire la prova successiva con il GPS compromesso senza che
nulla lo dica.

---

## Attacco al GNSS: cosa dicono le misure

Il progetto è nato con l'affermazione di essere resistente al *GPS denial*.
Verificarla ha prodotto tre risultati, tutti misurati prima di scrivere il nodo
e riportati qui come sono, non come ci si aspettava.

**1. Negare il GPS non degrada la navigazione.** Con il ricevitore spento per
60 secondi, lo scarto fra la stima di posizione dell'autopilota e la verità a
terra di Gazebo resta **sotto il metro e mezzo**, lo stesso valore che si misura
con il GPS sano. Le ragioni appartengono all'autopilota, non a questo progetto:
la quota viene dal barometro (`EK3_SRC1_POSZ = 1`), la navigazione inerziale non
deriva in modo apprezzabile su tempi di questo ordine, e una missione dura
un minuto.

**2. Falsificare il GPS non inganna l'EKF.** Un offset di 22 m, applicato sia a
gradino sia a rampa, si vede sul fix grezzo — verificato su
`/mavros/global_position/raw/fix`, la latitudine passa da −35.3630816 a
−35.362882 — ma la stima **non lo segue**. L'EKF rifiuta la misura incoerente
con la propria predizione, che è il comportamento corretto di un filtro ben
fatto.

**3. La via di iniezione funziona.** La scrittura di un parametro
dell'autopilota via `/mavros/param/set` risponde in 9-97 ms, quindi un nodo può
pilotarla senza rischio di bloccarsi.

Ne segue che **l'affermazione originale non è dimostrabile in questa
configurazione**, non perché il sistema sia fragile ma perché la negazione del
GNSS non produce, qui, un effetto a cui resistere. Cancellare l'affermazione
sarebbe però sbagliato quanto tenerla: quello che si può dimostrare, e che è
stato misurato, è che **l'inseguimento visivo non dipende dal GNSS**. Si esegue
la stessa missione due volte, con GPS sano e con GPS negato per tutta la durata,
e si confrontano durata dell'aggancio, distanza mediana e frazione di fotogrammi
con bersaglio:

```bash
prova.sh gnss_off 50
# ...riavvio con gnss_denial:=true gnss_modo:=negazione...
prova.sh gnss_negato 50
metriche.py confronta /ws/metrics/*gnss_off*.csv /ws/metrics/*gnss_negato*.csv
```

Il confronto, su 50 secondi simulati per prova, con il ricevitore spento nel
99.2% dei campioni della seconda:

| Indicatore | GPS sano | GPS negato |
|---|---|---|
| Sequenza delle fasi | `PATTUGLIAMENTO → AGGANCIO` | identica |
| Durata dell'aggancio | 40.8 s | 42.0 s |
| Distanza mediana | 2.21 m | 2.10 m |
| Distanza minima | 0.28 m | 0.25 m |
| Bersaglio rilevato | 82.6% | 82.9% |
| Scarto stima-verità, mediano | 0.15 m | 0.15 m |
| Scarto stima-verità, massimo | 1.24 m | 1.20 m |

Le due colonne coincidono entro la banda di ripetibilità misurata (±3%).
L'inseguimento visivo non usa il GNSS e la prova lo mostra; l'ultima riga dice
però anche l'altra metà della storia, ovvero che **nemmeno la navigazione
dell'autopilota degrada**, ed è la ragione per cui la resistenza alla negazione
non è dimostrabile qui: non c'è nulla a cui resistere.

Questa è la formulazione che le misure sostengono, e sostituisce quella che il
progetto dichiarava senza prove.

---

## Misura e ripetibilità

Le cifre citate in questo documento nascevano da script Python scritti sul
momento e mai salvati: nessun terzo poteva riprodurle, e due prove della stessa
configurazione non erano confrontabili perché a cambiare era anche lo strumento.
Gli elementi che chiudono la lacuna sono quattro.

**Tempo di simulazione.** Tutti i nodi girano con `use_sim_time` attivo e
`/clock` pontato da Gazebo (`tracking.launch.py`). Prima gli intervalli erano
misurati sull'orologio di parete, cosa che vale solo perché il SITL gira a
velocità 1 e non a lockstep — un'assunzione mai dichiarata. Attenzione: con
`use_sim_time` i timer dei nodi non partono finché `/clock` non pubblica, cioè
finché Gazebo non è in esecuzione. Per lanciare i nodi da soli:

```bash
ros2 launch drone_tracking tracking.launch.py use_sim_time:=false
```

**Disturbo ripetibile.** `jammer_node` parte da un seme fisso (`seed`, default
42), quindi due prove ricevono la stessa sequenza di rumore.

**Una prova in un comando.** `prova.sh` fa la sequenza completa — fotografa la
configurazione, decolla se serve, avvia la missione, attende, riassume:

```bash
prova.sh baseline 150
```

Una procedura digitata a mano cambia ogni volta di qualche dettaglio, e quel
dettaglio finisce nei numeri. `metrics_node` apre un file nuovo a ogni avvio di
missione, quindi una prova corrisponde sempre a un file.

**Dati della prova.** `metrics_node` scrive il CSV; `metriche.py` lo legge:

```bash
metriche.py riassumi /ws/metrics/metrics_20260904_181500.csv
metriche.py confronta /ws/metrics/prova_A.csv /ws/metrics/prova_B.csv
```

`riassumi` dà durata degli agganci, distanza mediana e media, frazione di
campioni con bersaglio, tempo per fase, ritmo della percezione. `confronta`
verifica la ripetibilità di due prove gemelle.

**Quanto sono ripetibili, in concreto.** Due prove con la stessa configurazione
e lo stesso seme, da stack riavviato (misura del 4 settembre 2026, 60 s
simulati ciascuna):

| | A | B |
|---|---|---|
| Sequenza di fasi | `PATTUGLIAMENTO → AGGANCIO` | identica |
| Durata dell'aggancio | 52.4 s | 53.8 s |
| Distanza mediana | 2.66 m | 2.51 m |
| Distanza media | 5.33 m | 5.32 m |
| Bersaglio rilevato | 85.5% | 85.0% |
| Fattore di tempo reale | 0.30x | 0.26x |

Le grandezze aggregate coincidono entro il 3%, ma le **posizioni istantanee
no**: allineando le due prove riga per riga, la posizione del drone differisce
in media di 6 m. Non e divergenza della dinamica, e sfasamento — la missione
parte a un istante diverso e da lì tutto slitta. La ripetibilità del progetto e
quindi **in distribuzione, non in traiettoria**: si confrontano durata degli
agganci, distanze mediane e frazioni di visibilità, non gli istanti uno per
uno.

Per ridurre lo sfasamento, `target_mover_node` riporta il bersaglio al punto di
partenza dell'orbita quando la missione lascia `ATTESA`: senza questo, ogni
prova trovava il bersaglio in un punto diverso dell'orbita a seconda di quanto
era durato il decollo.

**Prove automatiche.** Le correzioni piu facili da rompere in seguito senza
accorgersene — watchdog, coasting, scalatura di `Q`, soglia di ricerca — hanno
una prova che le esercita direttamente, senza far volare nulla:

```bash
colcon test --packages-select drone_tracking && colcon test-result --verbose
```

Provocare quei casi in simulazione richiederebbe di fermare un nodo a mano o di
aspettare che scada una soglia; qui il tempo si simula riavvolgendo gli istanti
registrati dai nodi.

**Configurazione della prova.** Il CSV dice come è andata, non con che taratura,
e i parametri sono modificabili a caldo. Prima di una prova conviene quindi
fotografarli:

```bash
salva_config.sh nome_della_prova
```

che scrive `<marca>_<nome>.params.yaml` accanto ai CSV. È il motivo per cui le
costanti tarate sono diventate parametri ROS: un valore letterale nel codice non
dice nulla su una prova già conclusa.

Per dare un'etichetta al file di una prova:

```bash
ros2 launch drone_tracking tracking.launch.py etichetta_config:=gimbal_off seed:=7
```

---

## Avvio manuale (installazione nativa)

Da usare sulla VM/macchina Ubuntu dove ArduPilot e Gazebo sono installati
localmente. Sostituire i percorsi con i propri. L'ordine è obbligatorio.

### Preparazione, una volta sola

**Symlink degli asset Gazebo.** Il repository versiona mondo e modello sotto
`sim/`, ma `gz sim` li carica da `ardupilot_gazebo/`: senza questo passaggio
`git pull` aggiorna file che la simulazione non legge. Vedi *Problemi noti*.

```bash
cd "$HOME/Desktop/Progetto Drone/ardupilot_gazebo/worlds"
mv iris_runway.sdf iris_runway.sdf.bak
ln -s "$HOME/Desktop/Progetto Drone/drone_tracking_ws/sim/worlds/iris_runway.sdf" .
```

```bash
cd "$HOME/Desktop/Progetto Drone/ardupilot_gazebo/models/iris_with_ardupilot"
mv model.sdf model.sdf.bak
ln -s "$HOME/Desktop/Progetto Drone/drone_tracking_ws/sim/models/iris_with_ardupilot/model.sdf" .
```

**Parametri aggiuntivi del SITL** — non c'è nulla da creare. Stanno in
[docker/sitl-defaults.parm](docker/sitl-defaults.parm), versionato nel
repository, e il comando T2 lo carica direttamente: container e VM leggono lo
stesso file, quindi `git pull` aggiorna anche la taratura del volo. Il nome della
cartella `docker/` è storico, il contenuto non ha nulla di specifico del
container.

Due note su quel file. `ARMING_CHECK` **non esiste** in questa versione di
ArduPilot e viene ignorato in silenzio, lasciando tutti i controlli attivi: il
parametro giusto è `ARMING_SKIPCHK`, con logica inversa, dove `1` significa
"salta tutto". E i `SIM_*_RND` azzerano il rumore degli IMU simulati, perché
quando la fisica singhiozza i tre giroscopi divergono e l'arming viene rifiutato
con `Arm: Gyros inconsistent`, un controllo che `ARMING_SKIPCHK` non copre.

### Prima di ogni prova

```bash
cd "$HOME/Desktop/Progetto Drone/drone_tracking_ws"
git pull
colcon build --packages-select drone_tracking
source install/setup.bash
```

**T1 — Gazebo**

```bash
export LIBGL_ALWAYS_SOFTWARE=1
gz sim -v4 -r "$HOME/Desktop/Progetto Drone/ardupilot_gazebo/worlds/iris_runway.sdf"
```

**T2 — ArduPilot SITL**

```bash
cd "$HOME/Desktop/Progetto Drone/ardupilot"
./build/sitl/bin/arducopter --model JSON --speedup 1 \
  --sim-address=127.0.0.1 --sim-port-in=9013 --sim-port-out=9012 \
  -I0 --defaults Tools/autotest/default_params/gazebo-iris.parm,"$HOME/Desktop/Progetto Drone/drone_tracking_ws/docker/sitl-defaults.parm"
```

**T3 — MAVProxy**

```bash
source ~/venv-ardupilot/bin/activate
mavproxy.py --master tcp:127.0.0.1:5760 \
  --out 127.0.0.1:14550 --out udp:127.0.0.1:14555 --console
```

**T4 — MAVROS2**

```bash
source /opt/ros/jazzy/setup.bash
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://:14555@127.0.0.1:14556 \
  -p target_system_id:=1 -p target_component_id:=1 \
  -p plugin_denylist:=[distance_sensor]
```

MAVROS si collega all'**uscita UDP di MAVProxy**, non a una porta TCP del SITL:
vedi *Problemi noti* per il motivo, che è tutt'altro che ovvio.

L'ultimo parametro esclude il plugin `distance_sensor`, che altrimenti riempie i
log di `DS: no mapping for sensor id: 0, type: 4, orientation: 25` più volte al
secondo. Omettendolo il sistema funziona lo stesso, ma i log diventano
illeggibili.

**T5 — Nodi ROS 2**

```bash
export RCUTILS_COLORIZED_OUTPUT=1
cd "$HOME/Desktop/Progetto Drone/drone_tracking_ws"
source install/setup.bash
ros2 launch drone_tracking tracking.launch.py
```

Il launch file avvia i sei nodi **e** il `parameter_bridge` di `ros_gz` che porta
`/drone/camera/image_raw` da Gazebo a ROS 2. Non serve quindi lanciare un secondo
ponte a mano: due `parameter_bridge` sullo stesso topic si sovrappongono, e se il
secondo è dichiarato bidirezionale (`@gz.msgs.Image` invece di `[gz.msgs.Image`)
rimanda anche i messaggi da ROS 2 verso Gazebo.

**T6 — Decollo** (nella console MAVProxy)

> **Passaggio obbligatorio.** Saltandolo il drone resta a terra: `mission_node`
> comincia a pubblicare i setpoint di posizione, ArduPilot ruota per allinearsi
> allo yaw richiesto — sembra "girare su se stesso" — ma in GUIDED non decolla
> senza un comando esplicito, quindi non raggiunge mai i waypoint.

```
mode guided
arm throttle
takeoff 12
```

I controlli di arming sono già disabilitati da `ARMING_SKIPCHK 1`, caricato
all'avvio del SITL da `docker/sitl-defaults.parm`. Se l'arming viene rifiutato con
`Arm: Gyros inconsistent`, quel file non è stato caricato: si può impostare il
parametro a mano con `param set ARMING_SKIPCHK 1` prima di `arm throttle`.

**T7 — Avvio missione**

```bash
ros2 topic pub --once /mission/avvia std_msgs/msg/Bool "data: true"
```

La quota di decollo dev'essere **12 m**, coerente con i waypoint. La quota entra
anche nella conversione da coordinate immagine a metri fatta dal controller,
quindi volare a una quota molto diversa cambia la scala dell'errore — non i
guadagni, che sono in unità fisiche, ma l'ampiezza dell'area inquadrata.

---

## Topics ROS 2

### Catena di percezione

| Topic | Tipo | Descrizione |
|---|---|---|
| `/drone/camera/image_raw` | `sensor_msgs/Image` | Feed telecamera dal ponte ros_gz |
| `/target/position` | `geometry_msgs/Point` | Posizione grezza dal detector (`z` = area) |
| `/target/jammed_position` | `geometry_msgs/Point` | Posizione corrotta dal jammer |
| `/target/tracked_position` | `geometry_msgs/Point` | Stima filtrata dal Kalman |
| `/target/debug_image` | `sensor_msgs/Image` | Frame annotato per il debug |

### Guerra elettronica

| Topic | Tipo | Descrizione |
|---|---|---|
| `/gps/jammed` | `std_msgs/Bool` | Stato del jamming GPS |
| `/rf/noise_level` | `std_msgs/Float32` | Intensità rumore RF (0.0 – 1.0) |

### Controllo e missione

| Topic | Tipo | Descrizione |
|---|---|---|
| `/mission/avvia` | `std_msgs/Bool` | Trigger di avvio del pattugliamento |
| `/mission/stato` | `std_msgs/String` | Fase corrente della missione |
| `/tracker/reset` | `std_msgs/Bool` | Reset forzato del filtro |
| `/drone/cmd_vel` | `geometry_msgs/Twist` | Comandi di velocità (solo debug) |

### Interfaccia MAVROS

| Topic | Tipo | Direzione |
|---|---|---|
| `/mavros/setpoint_velocity/cmd_vel_unstamped` | `geometry_msgs/Twist` | ← controller (fase AGGANCIO) |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | ← mission (PATTUGLIAMENTO, RICERCA) |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | → mission, target_mover |
| `/mavros/global_position/rel_alt` | `std_msgs/Float64` | → controller |

I subscriber su topic MAVROS usano QoS **BEST_EFFORT** con `depth=1`: MAVROS
pubblica con QoS sensor-like e un subscriber RELIABLE non riceverebbe nulla.

---

## Monitoraggio

```bash
ros2 run rqt_image_view rqt_image_view
```

Selezionare `/target/debug_image` per vedere il rilevamento in tempo reale.

```bash
ros2 topic echo /mission/stato
```

```bash
ros2 run rqt_plot rqt_plot /target/position/x /target/jammed_position/x /target/tracked_position/x
```

Quest'ultimo è il grafico più significativo del progetto: mostra il segnale
pulito, quello corrotto dal jammer e la ricostruzione del Kalman sovrapposti.

---

## Note tecniche e problemi noti

**Lo stack parte senza simulatore, e nessuno lo dice** — se `start_all.sh` viene
invocato da una shell che ha gia caricato ROS 2 (`docker compose exec sim bash
-lc 'start_all.sh'`, oppure entrando nel container e lanciandolo da li), il
comando `gz sim` **stampa l'elenco dei sottocomandi disponibili e termina con
esito zero**. Motivo: il CLI `gz` scopre i propri sottocomandi dai file di
configurazione elencati in `GZ_CONFIG_PATH`, e il source di ROS 2 sovrascrive
quella variabile con i soli percorsi dei pacchetti vendored
(`gz_transport_vendor`, `gz_msgs_vendor`), dove `sim` non esiste. L'ambiente
passa al server tmux e da questo a tutte le finestre.

Il guasto è insidioso perché tutto il resto funziona: MAVROS si avvia, i sette
nodi partono, `ros2 node list` li elenca tutti, nessun log segnala un errore.
Con `use_sim_time` i timer restano semplicemente fermi e la missione non fa
nulla. Sintomo diagnostico: `/clock` non pubblica, e il pannello `gazebo` di
tmux mostra un prompt invece del log del simulatore.

`start_all.sh` rimette ora il percorso di sistema in testa alla variabile:

```bash
export GZ_CONFIG_PATH="/usr/share/gz${GZ_CONFIG_PATH:+:$GZ_CONFIG_PATH}"
```

**Telecamera** — link `camera_link` fissato a `base_link`, pose
`0.1 0 -0.05 0 1.5708 0`: puntata a **nadir**. Le versioni precedenti la
inclinavano in avanti (1.047 rad = 60°, documentata erroneamente come 45°). A
nadir "bersaglio al centro dell'immagine" coincide con "drone sopra il
bersaglio", che è l'obiettivo della missione; con l'asse inclinato in avanti il
drone doveva invece mantenere una distanza di stallo di ~7 m e non poteva mai
sovrastare il bersaglio. Con FOV orizzontale di 60° a 12 m di quota
l'inquadratura copre ~14 m di terreno.

Attenzione: cambiare l'inclinazione **non** riduce l'accoppiamento fra assetto e
immagine — una rotazione del corpo trasla l'inquadratura della stessa quantità
qualunque sia il puntamento. Per quello serve la compensazione d'assetto nel
controller.

**Plugin `RosCamera` inerte** — `iris_with_ardupilot/model.sdf` contiene un blocco
`gz-sim-ros-camera-system`: quel plugin **non esiste** in Gazebo Harmonic, che lo
ignora con un errore a console. Il feed arriva a ROS 2 unicamente tramite il tag
`<topic>/drone/camera/image_raw</topic>` del sensore, raccolto dal
`parameter_bridge` nel launch file. Il blocco può essere rimosso senza effetti.

**Convenzione del campo `z`** — `Point.z` non è una coordinata: trasporta l'area
del contorno e serve da flag di visibilità. Modificando i nodi va preservata,
perché è ciò che permette di distinguere il bersaglio centrato dal bersaglio
assente.

**Accelerazione 3D assente in VM** — servono `LIBGL_ALWAYS_SOFTWARE=1` e, per
ottenere una fisica fluida, la modalità server-only (`gz sim -s`). Nel container
è il default (`HEADLESS=1`).

**Conflitto sulla porta 9002** — la porta usata di default per il canale
ArduPilot ↔ Gazebo viene occupata da un processo Ruby interno a Gazebo. Il
progetto usa 9012/9013.

## Due orologi, non uno

È la chiave per capire il resto di questa sezione. Nel sistema convivono due
famiglie di nodi con nature diverse.

### Anzi, tre: il tempo simulato non scorre come quello di parete

Prima di distinguere i nodi a timer da quelli a callback va detta una cosa che
riguarda tutti e due i tipi. Senza accelerazione grafica Gazebo **non tiene il
passo del tempo reale**: misurato con `metriche.py` su questa macchina, il
fattore e **0.23**, cioe un secondo simulato richiede oltre quattro secondi di
orologio. Il fattore non e una costante del progetto: dipende dal carico della
macchina, ed e per questo che va riportato in ogni prova.

Finche i nodi misuravano gli intervalli sull'orologio di parete, quel rapporto
si infilava in ogni grandezza calcolata a partire da un `dt`, con conseguenze
che sono state prese per difetti del controllo:

| Grandezza | Effetto del `dt` di parete |
|---|---|
| Velocita del bersaglio | `target_mover_node` integrava il moto sul `dt` di parete, quindi il bersaglio si spostava di ~4.3 volte la velocita nominale. Le prove fatte contro "un bersaglio a 1.2 m/s" avevano di fronte un bersaglio a circa 5 m/s simulati. |
| Velocita stimata dal Kalman | Il filtro divideva lo spostamento per un `dt` ~4.3 volte troppo grande, sottostimando della stessa quantita la velocita del bersaglio. |
| Soglie in secondi | `soglia_avvia_ricerca_s`, `ritardo_evasione_s` e le altre valevano circa un quarto del dichiarato in tempo simulato, con un fattore che cambiava fra una prova e la successiva. |

Da qui la scelta di `use_sim_time` come primo intervento in assoluto: non e una
raffinatezza formale, e la condizione perche i parametri in secondi e in metri
al secondo significhino quello che dicono. Le prove precedenti a questa
correzione restano valide come osservazioni, ma i valori di velocita del
bersaglio che riportano vanno letti moltiplicati per il fattore di allora, che
non e stato registrato — un'altra ragione per cui `metrics_node` lo scrive
adesso in ogni riga.

**Guidati da timer** — battono a frequenza fissa, decisa da loro soli:
`mission_node` a 2 Hz (`create_timer(0.5, …)`), `jammer_node`, la
ripubblicazione di `controller_node` e `target_mover_node` a 10 Hz.

**Guidati da callback** — non hanno frequenza propria: ereditano quella di chi
sta a monte. E a monte dell'intera catena di percezione c'è la telecamera di
Gazebo, l'unico elemento la cui velocità dipende dal carico della macchina anziché
da una costante.

Un nodo a callback può solo *perdere* messaggi, mai crearne, quindi il ritmo cala
scendendo la catena. Misure simultanee su 20 s di missione:

| Topic | Frequenza | Note |
|---|---|---|
| `/drone/camera/image_raw` | 15.5 Hz | sorgente, limitata dal rendering |
| `/target/position` | 15.5 Hz | detector, 1:1 coi frame |
| `/target/jammed_position` | 13.5 Hz | limitatore del jammer a 50 ms |
| `/target/tracked_position` | 13.5 Hz | nessuna perdita |
| `/mission/stato` | 2.0 Hz | orologio indipendente |

I 2 Hz di `/mission/stato` non hanno alcun rapporto con i 15 Hz del detector: non
sono lo stesso orologio. Confonderli è stata l'origine di diversi bug, ora
corretti — vedi sotto.

**Tempi in secondi, non in conteggi di messaggi** — tutti i ritardi e i `dt` sono
ora misurati sull'orologio, non contando messaggi ricevuti. In precedenza erano
costanti tarate su un ipotetico 10 Hz che quasi nessun topic rispetta, con effetti
concreti:

| Costante | Comportamento reale prima | Ora |
|---|---|---|
| Ritardo di evasione | 50 conteggi su un topic a 2 Hz → **25 s** invece di 5 | `ritardo_evasione_s = 10.0` |
| `dt` del Kalman | fisso a 0.1 con ingresso fra 5 e 15 Hz | ricavato dai tempi reali |
| Derivata del controller | divisione per 0.1 fisso | divisione per il `dt` misurato |
| Soglia di avvio ricerca | 20 frame → fra 1.5 e 4 s secondo il carico | `soglia_avvia_ricerca_s` |
| Espansione della spirale | 0.002 per chiamata a 2 Hz = **4 mm/s** | `ricerca_vel_espansione = 0.4` m/s |

**La spirale di ricerca non si allargava** — è il caso più estremo dello stesso
errore. `ricerca_espansione += 0.002` a ogni chiamata, su un timer a 2 Hz, dà
**4 millimetri al secondo**: per passare da 3 a 25 m di raggio servivano
**92 minuti**. In pratica non era una spirale ma un cerchio fisso di raggio 3 m,
mentre il bersaglio in fuga si allontanava a 1.2 m/s. Il drone entrava in
`RICERCA` e non ritrovava più nulla.

Ora l'espansione è **0.4 m/s** e la velocità angolare **0.35 rad/s**: un giro
dura ~18 s e lascia ~7 m fra un braccio e il successivo, meno dei ~15 m
inquadrati a 12 m di quota, quindi la spirale non salta porzioni di terreno.
Da 3 a 25 m di raggio in 55 secondi. Superato `ricerca_raggio_max = 25.0` la
ricerca è dichiarata fallita e la missione torna a `PATTUGLIAMENTO`, invece di
allargarsi indefinitamente allontanandosi dall'area di interesse.

Effetto misurato su 100 s di missione:

| | Prima | Dopo |
|---|---|---|
| Tempo in `AGGANCIO` | 68% | **99.5%** |
| Distanza mediana | 12.5 m | 9.6 m |
| Riagganci dopo perdita | mai | 16.5 s e 0.9 s nei due casi osservati |

**Il bersaglio girava a un terzo della velocità prevista** — `target_mover_node`
comandava la posa lanciando il comando esterno `gz service` e **attendendone** la
fine. Una chiamata costa ~360 ms (misurato), quindi il timer dichiarato a 10 Hz
girava in realtà a ~2.8 Hz, e i parametri di moto erano di fatto tarati contro
quel timer strozzato.

Ora il nodo usa i **binding Python di gz-transport** (`python3-gz-transport13`),
che riusano un nodo di trasporto persistente: **0.4 ms per richiesta**, contro i
360 ms del CLI. Le velocità sono inoltre espresse in unità al secondo e integrate
sul `dt` reale, quindi il moto non dipende più dalla frequenza del timer. Se i
binding non sono installati il nodo ricade sul comando esterno, avvisando che
l'aggiornamento della posa scenderà a ~3 Hz.

**Il bersaglio scivolava via dalla traiettoria** — il modello nel mondo era
dinamico, quindi fra un comando di posa e il successivo la fisica se ne
impossessava: una sfera senza attrito di rotolamento accumulava velocità e
rotolava lentamente fuori dal percorso previsto, tanto da non farsi mai trovare
dal drone al primo passaggio. Ora è dichiarato `<static>true</static>`: il moto è
interamente comandato da `target_mover_node` e la fisica non lo tocca. La quota è
stata portata da 0.5 a 0.3 m, pari al raggio della sfera, così poggia a terra
invece di restare sospesa.

**MAVROS riempiva i log di errori sul sensore di distanza** — ArduPilot invia
messaggi `DISTANCE_SENSOR` dal rangefinder simulato, e il plugin `distance_sensor`
di MAVROS li rifiuta più volte al secondo perché non ha una mappatura configurata:

```
[ERROR] [mavros.distance_sensor]: DS: no mapping for sensor id: 0, type: 4, orientation: 25
```

Il progetto non usa il rangefinder, quindi il plugin viene escluso all'avvio con
`-p plugin_denylist:=[distance_sensor]`. Resta una sola riga informativa,
`Plugin distance_sensor ignored`. Si è preferito questo a disabilitare il
rangefinder lato ArduPilot, che avrebbe alterato il comportamento di volo.

**Il tracker perdeva un messaggio a ogni riacquisizione** — sul frame di
acquisizione il nodo usciva senza pubblicare. Sotto jamming, dove le
riacquisizioni sono continue, questo costava il **19%** dei messaggi
(9.2 Hz in uscita contro 11.4 in ingresso). Ora la posizione appena acquisita
viene pubblicata subito, e la catena non perde più nulla.

**Il tracker marcava le proprie predizioni come "bersaglio assente"** — durante
una perdita di segnale pubblicava la stima di Kalman ricopiando `z` dal messaggio
in ingresso, che vale 0, cioè il codice convenzionale di assenza. I nodi a valle
non ne soffrivano perché decidono su `x`/`y`, ma chiunque seguisse la convenzione
documentata avrebbe scartato stime valide. Ora `z` porta l'ultima area valida.

**Due copie degli asset Gazebo** — il container costruisce l'immagine a partire da
`sim/`, mentre l'avvio manuale (T1) carica i file da
`ardupilot_gazebo/worlds` e `ardupilot_gazebo/models`. Sono percorsi distinti con
gli stessi nomi: `git pull` aggiorna solo i primi, mentre Gazebo legge i secondi.
Il file corretto finisce sul disco e resta inutilizzato, e le correzioni sembrano
non avere effetto.

**La soluzione definitiva è un symlink**, da fare una volta sola. Non richiede di
cambiare il comando di avvio né di ricopiare nulla a ogni aggiornamento:

```bash
cd "$HOME/Desktop/Progetto Drone/ardupilot_gazebo/worlds"
mv iris_runway.sdf iris_runway.sdf.bak
ln -s "$HOME/Desktop/Progetto Drone/drone_tracking_ws/sim/worlds/iris_runway.sdf" .
```

```bash
cd "$HOME/Desktop/Progetto Drone/ardupilot_gazebo/models/iris_with_ardupilot"
mv model.sdf model.sdf.bak
ln -s "$HOME/Desktop/Progetto Drone/drone_tracking_ws/sim/models/iris_with_ardupilot/model.sdf" .
```

Da quel momento le due copie sono lo stesso file e `git pull` aggiorna davvero la
simulazione. Per controllare che la copia usata da Gazebo sia allineata:

```bash
grep -c "<static>true</static>" "$HOME/Desktop/Progetto Drone/ardupilot_gazebo/worlds/iris_runway.sdf"
```

```bash
grep "update_rate" "$HOME/Desktop/Progetto Drone/ardupilot_gazebo/models/iris_with_ardupilot/model.sdf"
```

Il primo deve restituire `1` (sfera statica, altrimenti rotola via da sola), il
secondo `<update_rate>30</update_rate>`. Se i valori non corrispondono, la
simulazione sta girando su asset vecchi e le correzioni del repository non hanno
effetto.

**L'immagine sobbalza, ma il drone è fermo** — con il rendering software i frame
non arrivano a cadenza regolare, e a schermo l'effetto è un video che "salta".
Non è un'oscillazione del velivolo: la verità di Gazebo, campionata a 30 Hz dal
topic delle pose, dà in hover **roll std 0.26°** ed escursione ±0.4°, con pitch
praticamente nullo. Il drone è stabile.

Attenzione a non farsi ingannare da `/mavros/imu/data`: pubblica a ~1.6 Hz e
campionarlo suggerisce oscillazioni di 0.4 rad/s che non esistono — è aliasing.
Per giudicare l'assetto va usato il topic delle pose di Gazebo, non l'IMU via
MAVLink.

Il rimedio misurato è abbassare `<update_rate>` della telecamera da 30 a **15**:
chiedere una frequenza irraggiungibile fa mancare al renderer ogni scadenza, e i
frame vengono consegnati quando capita. Misure su 20 s:

| | `update_rate` 30 | `update_rate` 15 |
|---|---|---|
| Frequenza effettiva | 11.4 Hz | 10.9 Hz |
| Deviazione standard | 31.2 ms | **14.2 ms** |
| Intervallo peggiore | 397.6 ms | **179.4 ms** |
| Jitter relativo | 36.4% | **17.1%** |

Il jitter si dimezza senza perdere frequenza. Il residuo dipende dal
rasterizzatore software: sparisce con accelerazione grafica.

**Nel repository il valore è comunque 30**, con la misura riportata nel commento
del modello: la scelta è deliberata, perché su una macchina con GPU i 30 Hz sono
raggiungibili e il jitter non si presenta, mentre abbassare il valore
penalizzerebbe anche quel caso. Chi gira in headless senza accelerazione e vuole
un'immagine più regolare può portarlo a 15 in
`sim/models/iris_with_ardupilot/model.sdf`.

**Frame rate della telecamera nel container** — il sensore dichiara
`<update_rate>30</update_rate>`, ma in headless senza GPU Gazebo renderizza via
rasterizzatore software e il topic `/drone/camera/image_raw` resta molto sotto:
misurato fra **5 e 11 Hz** su WSL2, a seconda del carico della macchina. Il
tracking funziona comunque: il filtro di Kalman ricava il proprio `dt`
dall'intervallo reale fra due misure, e la matrice del rumore di processo viene
ricostruita su quel `dt`, quindi una frequenza bassa allarga l'incertezza invece
di falsare la predizione. Resta il fatto che meno fotogrammi al secondo
significano meno informazione: per confrontare due prove conviene verificare che
girassero allo stesso ritmo, colonna `det_hz` del CSV di `metrics_node` oppure

```bash
ros2 topic hz /drone/camera/image_raw
```

**Versione del firmware** — ArduPilot non pubblica più un branch per ogni
release: i branch si fermano a `Copter-4.5`, le versioni successive esistono solo
come **tag**. Un binario compilato da `master` si dichiara `4.8.0-dev` pur non
corrispondendo ad alcuna release: è da lì che veniva il "v4.8.0" indicato in una
versione precedente di questo documento. Il container usa il tag stabile
`Copter-4.7.0`, modificabile con l'argomento di build `ARDUPILOT_REF`.

**MAVROS aborta con `Promise already satisfied`** — è un difetto di MAVROS 2, non
del progetto, ma si può evitare togliendone la causa scatenante. La sequenza nei
log è sempre questa:

```
CON: Lost connection, HEARTBEAT timed out.
VER: autopilot version service timeout
VER: command plugin service call failed!
failed to send response to /mavros/cmd/command (timeout)
terminate called after throwing an instance of 'std::future_error'
  what():  std::future_error: Promise already satisfied
```

Alla perdita di heartbeat la richiesta `AUTOPILOT_VERSION` va in timeout e MAVROS
chiude la promise con errore; quando la risposta arriva in ritardo prova a
chiuderla di nuovo, l'eccezione non è gestita e il processo aborta.

Il fattore scatenante è la perdita di heartbeat, favorita dal percorso
`SITL → MAVProxy → UDP → MAVROS` quando la macchina è carica per il rendering.

**Non collegare MAVROS direttamente al TCP del SITL per evitarlo.** Sembra la
soluzione ovvia — il SITL espone `SERIAL1` sulla 5762 e `SERIAL2` sulla 5763, e
il TCP non perde pacchetti — ma **non funziona**: ArduPilot regola gli stream
MAVLink per singola porta seriale, e su SERIAL1 quelli di posizione non sono
attivi. Il risultato misurato è insidioso perché parziale:

| Topic | Via MAVProxy (UDP) | Via TCP su SERIAL1 |
|---|---|---|
| `/mavros/state` | ok | ok, 0.86 Hz |
| `/mavros/local_position/pose` | ok | **nessun dato** |
| `/mavros/global_position/rel_alt` | ok | **nessun dato** |

MAVROS risulta connesso e l'heartbeat arriva, quindi tutto sembra a posto, ma
`mission_node` non riceve mai la posizione: `distanza_waypoint` restituisce
`inf`, il waypoint non è mai raggiunto e **il drone resta fermo sul punto di
decollo**. Il sintomo a log è inconfondibile:

```
[mission_node]: Waypoint 0/4 → (0,0,12)m dist:infm
```

Per usare davvero il TCP diretto occorrerebbe abilitare gli stream su quella
seriale (famiglia di parametri `SR1_*`, rinominata nelle versioni recenti), cosa
non verificata qui. Finché non lo è, si passa da MAVProxy.

Il crash di MAVROS resta quindi possibile sotto carico: se capita, basta
rilanciare MAVROS, gli altri nodi si riconnettono da soli senza toccare Gazebo o
il SITL.

**Controlli di arming** — il SITL con backend JSON fallisce spesso i controlli
sui sensori simulati, tipicamente con `Arm: Gyros inconsistent` o
`Accels inconsistent`: gli IMU simulati divergono quando la fisica singhiozza.

Il parametro che li disattiva è **`ARMING_SKIPCHK`**, non `ARMING_CHECK`.
Quest'ultimo non esiste in questa versione di ArduPilot e viene **ignorato in
silenzio**: si crede di aver disabilitato i controlli e invece sono tutti
attivi. `ARMING_SKIPCHK` ha inoltre logica inversa — `1` significa "salta
tutto", non "controlla tutto". Il valore corretto è caricato all'avvio da
`docker/sitl-defaults.parm`, caricato da entrambi gli ambienti tramite --defaults.

**Plugin `TrajectoryFollower` inerte** — il modello `bersaglio` dichiara un
`gz-sim-trajectory-follower-system` con cinque waypoint, ma quel percorso non ha
mai effetto e il bersaglio è mosso interamente da `target_mover_node`. Due motivi,
entrambi verificati sul sorgente di `gz-sim8`:

1. **Schema dei waypoint sbagliato.** Il plugin legge ogni `<waypoint>` come
   `math::Vector2d` dal valore dell'elemento, cioè si aspetta
   `<waypoint>20 20</waypoint>`. Il world usa la forma
   `<waypoint><time>…</time><pose>…</pose></waypoint>`, che è lo schema delle
   traiettorie degli *actor*: il valore dell'elemento è vuoto e `Get<Vector2d>()`
   restituisce il default, quindi tutti i waypoint collassano su `(0, 0)`.
2. **Meccanismo di attuazione diverso.** Il plugin muove il link con
   `AddWorldWrench` (forza e coppia), mentre `target_mover_node` chiama
   `gz service set_pose`, che riscrive la posa del modello 10 volte al secondo.
   Il teletrasporto azzera qualunque effetto della spinta a ogni tick.

Il risultato è corretto, ma per caso: il blocco `<plugin>` è di fatto codice morto
che applica una spinta parassita verso l'origine. Conviene rimuoverlo dal modello
`bersaglio` in `sim/worlds/iris_runway.sdf`, così il controllo resta in un posto
solo.

**Errori `gz service` silenziosi** — `target_mover_node` invoca `subprocess.run(cmd,
capture_output=True)` senza controllare `returncode`: se il servizio `set_pose`
fallisce (nome del mondo o del modello cambiato) il bersaglio resta fermo senza
alcun messaggio a log.

**Ordine di avvio** — il SITL deve trovare Gazebo già in ascolto sulle porte
JSON, e MAVROS deve trovare MAVProxy già attivo. `start_all.sh` gestisce i
ritardi automaticamente; a mano vanno rispettati i tempi tra un terminale e
l'altro.

---

## Contesto teorico

Il progetto simula scenari ispirati al conflitto russo-ucraino, dove:

- I **jammer GPS russi** (Krasukha, Murmansk-BN) rendono inaffidabile la
  navigazione GNSS su vaste aree
- I **droni FPV ucraini** usano visione artificiale per mantenere l'aggancio
  anche in zone di jamming attivo
- La **guerra elettronica cognitiva** sfrutta l'IA per riconoscere e falsificare
  firme elettromagnetiche

Il filtro di Kalman implementato replica il comportamento di un sistema INS
semplificato in coordinate immagine: predice la posizione del bersaglio durante
la perdita di segnale, aumenta l'incertezza di misura quando rileva rumore RF, e
corregge la stima quando il datalink viene ripristinato.

---

## Riferimenti

- ArduPilot SITL con Gazebo: https://ardupilot.org/dev/docs/sitl-with-gazebo.html
- ardupilot_gazebo: https://github.com/ArduPilot/ardupilot_gazebo
- MAVROS2: https://github.com/mavlink/mavros
- ROS 2 Jazzy: https://docs.ros.org/en/jazzy
- Kalman Filter: Welch & Bishop, *An Introduction to the Kalman Filter*, UNC Chapel Hill (2006)
- CSIS, *Quantum Sensing and the Future of Warfare* (2025)

---

## Licenza

Distribuito con licenza MIT. Vedi [LICENSE](LICENSE).

## Contribuire

Linee guida in [CONTRIBUTING.md](CONTRIBUTING.md), regole di convivenza in
[CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md). Per le vulnerabilità di sicurezza, e
per le precauzioni da adottare prima di portare l'architettura su hardware reale,
vedi [SECURITY.md](SECURITY.md).
