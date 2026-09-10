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

Quando il drone arriva sul waypoint `(150, 150)` e conferma il bersaglio per 5
rilevamenti consecutivi, lo stato passa ad `AGGANCIO`. Dieci secondi dopo il
bersaglio inizia la fuga a 15 m/s, e se il drone lo perde lo stato diventa
`RICERCA`: prima l'inseguimento nella direzione di fuga, poi la spirale.
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
| `RICERCA` | Bersaglio perso: inseguimento nella direzione di fuga, poi spirale. |

**Percorso di pattugliamento** — circuito quadrato a 50 m di quota, con soglia
di raggiungimento pari al parametro `soglia_waypoint`:

```
(0,0) → (150,0) → (150,150) → (0,150) → (0,0)
```

Il vertice `(150,150)` coincide con la zona in cui orbita il bersaglio. Se il
giro si chiude senza aggancio, il pattugliamento riparte dal waypoint 1.

**Aggancio** — richiede `frame_conferma_richiesti` **rilevamenti** consecutivi,
e solo sopra i 2 m di quota, per evitare falsi positivi durante il decollo. Sono
rilevamenti e non messaggi validi del tracker: quest'ultimo resta valido per
`soglia_perdita` fotogrammi dopo l'ultima misura, quindi un solo avvistamento ne
produrrebbe abbastanza da soddisfare qualunque soglia di conferma. Il conteggio
è sul flusso a valle del jammer, lo stesso che il filtro riceve in ingresso:
contare i rilevamenti puliti farebbe confermare un aggancio su un'informazione
che il filtro non ha mai avuto.

**Perdita e ricerca** — trascorso `soglia_avvia_ricerca_s` senza bersaglio in
`AGGANCIO`, la fase passa a `RICERCA`, che si svolge in due tempi.

Nel primo il drone vola dove il bersaglio sarebbe se avesse proseguito dritto,
estrapolando dalla posizione e dalla velocità che `controller_node` pubblica su
`/target/odometria`. Dura `durata_inseguimento_cieco_s`. È il tempo in cui si
usa l'unica informazione direzionale disponibile, e a velocità reali vale più di
qualunque strategia di copertura: la spirale si espande di pochi metri al
secondo mentre un veicolo in fuga ne percorre quindici.

Nel secondo si apre la spirale, **centrata sulla posizione extrapolata del
bersaglio e non su quella del drone** — a queste velocità le due differiscono di
decine di metri, e cercare attorno a sé significa cercare dove il bersaglio non
è. Il raggio parte da `ricerca_raggio` e cresce di `ricerca_vel_espansione` al
secondo fino a `ricerca_raggio_max`, oltre il quale la ricerca è dichiarata
fallita e la missione torna a `PATTUGLIAMENTO`.

Per tornare in `AGGANCIO` servono `frame_conferma_riaggancio` rilevamenti
consecutivi. Anche qui rilevamenti e non messaggi del tracker, e per un motivo
misurato: si veda la sezione sulla ricerca direzionale.

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

**La velocità stimata esce dal filtro.** Il filtro calcolava `vx, vy` fin
dall'inizio, ma nessuno li leggeva. Ora sono pubblicati su
`/target/tracked_velocity`, in coordinate immagine al secondo e **relativi** al
drone: l'immagine si muove anche quando a muoversi è il velivolo.

**L'intensità di rumore del modello è tarata sulla verità a terra.** Il valore
iniziale, 5.0, era stato scelto per riprodurre a `dt` nominale il termine di
velocità della vecchia matrice costante — un argomento di continuità con una
matrice che era essa stessa sbagliata. Confrontando la velocità stimata con
quella reale del bersaglio letta dal simulatore, la pendenza della regressione
vale:

| `intensita_rumore_accel` | Pendenza in avanti | Pendenza laterale |
|---|---|---|
| 5.0 | 2.36 | 1.51 |
| **0.5** | **1.03** | **0.86** |
| 0.05 | 0.55 | 0.55 |

A 5.0 la stima era gonfiata del doppio, a 0.05 il filtro insegue troppo
lentamente e la dimezza. Il difetto era invisibile finché la velocità non è
servita a qualcuno. La correlazione con la verità resta intorno a 0.6 in tutti
e tre i casi: quella è la qualità intrinseca della misura e non dipende da
questo parametro. La correzione ha migliorato anche l'inseguimento di per sé,
dal 73.2% all'84.0% di fotogrammi con bersaglio a 5.5 m/s di fuga.

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
| `kp_x`, `kp_y` | 2.0 | 1/s |
| `kd_x`, `kd_y` | 0.35 | — |
| `vel_max` | 20.0 | m/s (per asse) |
| `deadzone` | 2.0 | metri |
| `k_anticipo` | 0.0 | — (misurato, non conviene) |
| `finestra_velocita_s` | 1.0 | s (mediana della velocita stimata) |
| `campioni_velocita_min` | 5 | campioni sotto i quali la stima si disattiva |
| `vel_bersaglio_max` | 25.0 | m/s — limite fisico, non una taratura |
| `validita_velocita_s` | 2.0 | s per cui l'ultima velocita nota resta usabile |
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

**Guida predittiva, misurata e spenta.** Il controllo è reattivo: corregge lo
scarto osservato, non anticipa. Da qui l'errore a regime `velocità / kp` di
5.2. Il rimedio naturale è aggiungere al comando la velocità del bersaglio, che
il filtro stima già — con una precisazione che cambia il progetto del termine:
quella stima è **relativa**, e sommarla direttamente duplicherebbe il termine
derivativo, che usa la stessa grandezza. Per pareggiare la velocità del
bersaglio serve la sua velocità **assoluta**, cioè quella relativa più quella
del velivolo letta da `/mavros/local_position/velocity_local` (verificato
sperimentalmente che sia nel frame del mondo e non del corpo: lo scarto rispetto
alla verità a terra vale 0.29 m/s contro 0.43 m/s dell'ipotesi opposta).

Implementato così, contro un bersaglio in fuga a 5.5 m/s e con la stima già
corretta in scala:

| `k_anticipo` | Bersaglio rilevato | Distanza mediana | Durata aggancio |
|---|---|---|---|
| **0.0** | **84.0%** | **3.92 m** | 43.4 s |
| 0.4 | 77.6% | 4.21 m | 41.8 s |
| 0.7 | 82.0% | 4.06 m | 41.8 s |
| 1.0 | 74.0% | 6.42 m | 43.8 s |

Lo zero è il punto migliore, e i valori intermedi si equivalgono entro la
dispersione fra prove ripetute a questa velocità. La ragione non è il termine
in sé ma la qualità della stima: con correlazione 0.6 fra velocità stimata e
velocità vera, poco più di un terzo della varianza della stima è segnale, e un
anticipo somma l'intera stima al comando — rumore compreso — che costa più del
ritardo che elimina.

Il termine resta nel codice, parametrico e spento per default. Renderlo
conveniente richiede una stima migliore, non un guadagno diverso: la via
naturale è dare al filtro la velocità del velivolo come **ingresso noto**, così
che stimi direttamente la velocità assoluta del bersaglio invece di ricavarla
per differenza da un'immagine in cui i due moti sono sovrapposti.

Un timer dedicato a 10 Hz ripubblica il comando corrente su
`/mavros/setpoint_velocity/cmd_vel_unstamped`: ArduPilot esce dal controllo in
velocità se non riceve setpoint con continuità. Pubblica solo in fase `AGGANCIO`
e sopra 1 m di quota.

### target_mover_node

Muove il bersaglio in Gazebo — un parallelepipedo rosso di 4.0 × 2.0 × 1.6 m,
le dimensioni di un veicolo leggero — comandandone la posa tramite il servizio
`set_pose`, con due comportamenti:

| Fase | Comportamento | Velocità |
|---|---|---|
| `PATTUGLIO` | Orbita circolare attorno a `(150, 150)`, raggio `raggio_orbita` (40 m) | `velocita_angolare` (0.25 rad/s ≈ **10 m/s**, 36 km/h) |
| `EVASIONE` | Fuga in linea retta opposta al drone, per `durata_evasione_s` | `vel_evasione` a regime, raggiunta con rampa `accel_evasione` |

**Valori attuali.** La fuga è a `vel_evasione` = 15 m/s, cioè 54 km/h, raggiunti
con una rampa di 3 m/s²: cinque secondi per il regime, come un mezzo leggero su
sterrato. Dura `durata_evasione_s` = 20 s, che a quella velocità porta il
bersaglio a circa 260 m dal punto di partenza — molto più del semicampo
inquadrato, quindi la fuga mette davvero alla prova l'inseguimento invece di
svolgersi tutta dentro una sola inquadratura.

**Perché il limite non è la velocità del drone.** Il vincolo è l'**errore a
regime** del controllo proporzionale: inseguendo un bersaglio a velocità
costante, l'errore d'immagine si stabilizza intorno a `velocità_bersaglio / kp`.
La tabella seguente appartiene allo scenario precedente — orbita da 3 m a passo
d'uomo, quota 12 m, `kp = 4.0` su coordinate normalizzate — e resta qui perché
il ragionamento vale ancora, non i numeri:

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

**L'ingresso del filtro, non solo la sua uscita.** Le colonne `jam_x`, `jam_y`,
`jam_valido` registrano `/target/jammed_position`, cioè il rilevamento **dopo**
il disturbo — quello che il filtro riceve davvero — e `rumore_rf` il livello
dichiarato sul datalink, da cui dipende la matrice `R`. Mancavano, e senza di
esse la domanda «quanto serve il filtro» non è rispondibile con i dati:
confrontare la sua uscita con il rilevamento pulito misura l'errore residuo, non
il guadagno, perché l'alternativa al filtro non è il segnale pulito — che
nessuno possiede — ma quello disturbato.

Alla chiusura il nodo stampa un riepilogo (campioni, percentuale di fotogrammi
con bersaglio, distanza media, tempo per fase).

### gimbal_node

Stabilizza la telecamera comandando i due giunti della sospensione cardanica in
senso opposto all'assetto misurato. Il difetto che affronta è geometrico e non
di taratura: la telecamera era imbullonata al corpo, e un multirotore accelera
inclinandosi, quindi ogni comando di inseguimento produceva una rotazione che
traslava l'inquadratura indipendentemente da dove fosse il bersaglio. Più il
controllo era pronto, più il velivolo si inclinava, e prima il bersaglio usciva
dal campo — le due grandezze non si potevano ottimizzare separatamente.

La compensazione analitica in `controller_node` **resta al suo posto**: corregge
l'errore di controllo, non l'osservazione. Se l'inclinazione porta il bersaglio
fuori dai pixel, nessun calcolo lo recupera; e quando il giunto satura ai 45°
del proprio limite, il residuo torna a suo carico.

| Parametro | Default | Significato |
|---|---|---|
| `abilitato` | `true` | a `false` comanda zero: la telecamera si comporta come fissa |
| `guadagno` | 1.0 | frazione dell'assetto compensata, per misurare quanto conta |
| `limite_rad` | 0.7854 | limite meccanico del giunto, 45° |
| `timeout_posa_s` | 1.0 | oltre, il comando resta all'ultimo valore e l'anomalia è segnalata |

Due dettagli non ovvi del modello. Gli assi dei giunti sono dichiarati nel frame
del **modello** (`expressed_in="__model__"`) e non in quello del giunto: la
telecamera è ruotata di 90° per guardare a nadir, e senza quella precisazione
gli assi erediterebbero la rotazione, scambiando rollio e beccheggio. I link
della sospensione hanno massa 10 g e non zero: due corpi quasi privi di massa in
serie, comandati da un regolatore, sono una sorgente classica di instabilità
numerica.

Il regolatore è `gz-sim-joint-position-controller-system`, con
`use_velocity_commands` attivo: con corpi così leggeri un anello in coppia
oscilla, mentre un servo reale è comunque rigido in posizione. Verificato sul
banco a terra — comandati +0.30 e −0.20 rad, i giunti raggiungono esattamente
+0.30 e −0.20, sia dal lato Gazebo sia attraverso il ponte da ROS 2.

La configurazione di riferimento si ottiene con `gimbal:=false`, che lascia i
giunti fermi a zero: il confronto avviene così a parità di velivolo, di masse e
di dinamica, invece di confrontare due modelli diversi.

```bash
docker compose exec -e LAUNCH_ARGS="gimbal:=false" sim start_all.sh --detach
```

**Il gimbal da solo peggiora le cose.** È il risultato più istruttivo di questa
fase e vale la pena riportarlo. Attivando la sospensione senza toccare nient'
altro, l'accoppiamento crolla come previsto ma l'inseguimento degrada: bersaglio
rilevato dall'82.9% al 54.2%, distanza mediana da 3.70 a 10.54 m, e per la prima
volta la missione cade in `RICERCA`.

La causa è che due correzioni agivano sullo stesso effetto. Il gimbal stabilizza
l'immagine, e `controller_node` continuava a sottrarre l'assetto del **corpo**
da un'immagine in cui quell'assetto non compariva più: un errore fantasma
proporzionale all'inclinazione, cioè lo stesso difetto che la compensazione
doveva eliminare, col segno rovesciato. La grandezza corretta è l'assetto della
**telecamera**, che vale assetto del corpo più angolo del giunto — per questo il
controller si iscrive ai due topic di comando del gimbal. Senza gimbal l'angolo
è zero e la formula torna quella precedente; con il giunto in saturazione resta
il residuo, che è esattamente ciò che va ancora compensato.

Misure su fuga a 4 m/s, cinquanta secondi simulati per configurazione:

| | Telecamera fissa | Gimbal, doppia compensazione | Gimbal + residuo |
|---|---|---|---|
| Accoppiamento `pitch`/`det_y` | −0.858 | −0.317 | −0.222 |
| Accoppiamento `roll`/`det_x` | +0.702 | +0.199 | +0.217 |
| Bersaglio rilevato | 82.9% | 54.2% | 84.0% |
| Fornito dal filtro | 82.9% | 66.7% | 84.7% |
| Distanza mediana | 3.70 m | 10.54 m | 3.88 m |
| Durata aggancio | 42.0 s | 49.0 s + 9 s di ricerca | 45.8 s |

L'assetto spiegava il 74% della varianza della posizione del bersaglio
nell'immagine (r² = 0.74); con la sospensione ne spiega il 5%. A questa velocità
di fuga, però, le metriche di inseguimento **pareggiano** il riferimento invece
di migliorarlo: il vincolo dominante qui non è l'accoppiamento — il riferimento
teneva l'aggancio per tutta la prova — ma l'errore a regime del controllo
proporzionale descritto sopra.

La prova è stata ripetuta a 5.5 m/s, oltre il limite di velocità del velivolo,
dove il riferimento è in difficoltà:

| | Telecamera fissa | Gimbal + residuo |
|---|---|---|
| Accoppiamento `pitch`/`det_y` | −0.869 | −0.241 |
| Accoppiamento `roll`/`det_x` | +0.544 | +0.129 |
| Bersaglio rilevato | 73.2% | 74.3% |
| Fornito dal filtro | 81.3% | 83.0% |
| Distanza mediana | 3.86 m | 3.66 m |
| Durata aggancio | 41.8 s | 43.8 s |

Di nuovo l'accoppiamento scompare (r² da 0.76 a 0.06) e di nuovo le metriche di
inseguimento si muovono appena. La spiegazione sta nei numeri stessi: con una
distanza mediana di 3.9 m e un semicampo che a 12 m di quota copre 12 m al
suolo, la traslazione da assetto — 5.6 m equivalenti a 25° di inclinazione —
porta il bersaglio a circa 9.5 m, ancora dentro l'inquadratura. Finché il
bersaglio resta inquadrato comunque, eliminare l'accoppiamento non aggiunge
fotogrammi.

**Il valore della sospensione si vede alzando il guadagno.** Il progetto
sosteneva che aggressività e osservabilità non si possono ottimizzare
separatamente: più il controllo è pronto, più il velivolo si inclina, e prima il
bersaglio esce dall'inquadratura. Se la sospensione disaccoppia davvero le due
cose, quel prezzo deve smettere di esistere. La prova, con `kp_x` e `kp_y`
portati da 1.2 a 2.0 e fuga a 5.5 m/s:

| | Telecamera fissa | Gimbal |
|---|---|---|
| Sequenza delle fasi | `PATT → AGG → RICERCA → AGG` | `PATT → AGG` |
| Bersaglio rilevato | 53.1% | 80.6% |
| Fornito dal filtro | 64.8% | 82.8% |
| Distanza mediana | 8.58 m | 4.12 m |
| Agganci | 2 (26.4 e 12.8 s), perso una volta | 1 (44.4 s), mai perso |
| Accoppiamento `pitch`/`det_y` | −0.786 | −0.371 |

Confrontate con il guadagno nominale alla stessa velocità: senza sospensione
l'aumento fa **crollare** il rilevamento dal 73.2% al 53.1% e fa perdere
l'aggancio; con la sospensione lo fa **salire** dal 74.3% all'80.6% e l'aggancio
tiene per l'intera prova. È la dimostrazione diretta dell'affermazione
architetturale: il conflitto fra guadagno e campo visivo esiste solo con la
telecamera solidale al corpo.

**Cosa vale allora la sospensione.** Rende l'immagine una misura del bersaglio
invece che dell'assetto del velivolo — dimostrato — e con questo rimuove il
vincolo che teneva bassi i guadagni. Al guadagno nominale non paga nulla, e
anche questo va detto: il vincolo dominante a 1.2 non era l'accoppiamento. Paga
quando si sfrutta il margine che apre.

I default nel codice restano `kp = 1.2`, che è il valore sicuro **in entrambe le
configurazioni**: con `gimbal:=false` e `kp = 2.0` il sistema è peggiore di
quello attuale. Chi gira con la sospensione attiva può alzarlo senza
ricompilare:

```bash
ros2 param set /controller_node kp_x 2.0 && ros2 param set /controller_node kp_y 2.0
```

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

## Ricerca del bersaglio: cosa dicono le misure

La spirale funzionava contro un bersaglio a passo d'uomo e non poteva
funzionare contro un veicolo. Il motivo è aritmetico prima che sperimentale: si
espande di pochi metri al secondo attorno al punto di perdita mentre il
bersaglio se ne allontana a quindici, quindi non lo raggiungerà mai, qualunque
sia il raggio massimo.

**La misura di partenza.** Cinque prove a scala reale, con la spirale isotropa
centrata sulla posizione del drone:

| Prova | Esito della ricerca | Distanza iniziale → minima |
|---|---|---|
| `reale_kp2p0` | riagganciato dopo 32.2 s | 128 → 44 m |
| `reale_ff1p0` | fallita | 155 → 100 m |
| `reale_kp4p0` | fallita | 114 → **114** m |
| `reale_ff0p0` | fallita | 110 → **110** m |
| `scenario_50m` | fallita | 106 → **106** m |

Una su cinque, e in tre casi la distanza minima **coincide con quella
iniziale**: durante l'intera ricerca il drone non si è avvicinato nemmeno di un
metro. In `reale_ff0p0` è passata da 110 a 221 m con un massimo di 302: la
spirale portava il drone via dal bersaglio. L'unico successo non smentisce il
ragionamento, lo conferma — arriva quando la fuga è finita e il bersaglio ha
ripreso l'orbita, cioè quando ha smesso di allontanarsi.

**I numeri che dicono cosa serviva.** Due grandezze misurate delimitano il
problema. Alla dichiarazione di perdita il bersaglio è già a **106-155 m**, non
a pochi metri: la fase `AGGANCIO` sopravvive per inerzia — coasting del
controllo più predizione del filtro — mentre il bersaglio è fuori inquadratura
da un pezzo, e l'ultimo campione in `AGGANCIO` risulta a 107 m. In ricerca il
drone vola però a **20 m/s**, saturando `WP_SPD`, contro un bersaglio a 10-15:
il margine di recupero esiste, vale 5-10 m/s, e la spirale lo spende girando in
tondo. Poiché a 50 m di quota l'impronta a terra è larga 100 m, per rivedere il
bersaglio basta scendere sotto i ~40 m di distanza, cioè recuperarne 70: sono
sette-quattordici secondi di inseguimento nella direzione giusta.

**La modifica.** La ricerca diventa in due tempi. Nel primo il drone vola dove
il bersaglio sarebbe se avesse proseguito, estrapolando dalla posizione e dalla
velocità che `controller_node` pubblica su `/target/odometria`; nel secondo si
apre la spirale attorno al punto extrapolato invece che attorno al luogo della
perdita. La conversione da coordinate immagine a metri la pubblica il controllo
e non la rifà la missione: è la stessa formula, e due copie della stessa formula
divergono al primo che ne corregge una sola.

Anticipando la conclusione, perché le misure che seguono si leggano sapendo
dove portano: delle due metà, **il centro della ricerca sul bersaglio resta
acceso e l'estrapolazione no**. La prima non dipende dalla velocità stimata, la
seconda sì, e quella stima non è all'altezza — `durata_inseguimento_cieco_s`
vale quindi `0.0` per default. Il perché è quantificato più sotto.

Misurata su tre prove per configurazione, alternate, contro la stessa spirale
centrata però sulla posizione del bersaglio — quindi il confronto isola
l'estrapolazione, non l'intera modifica:

| Su tutti gli episodi di ricerca | direzionale | spirale |
|---|---|---|
| Riagganci | 75% | 60% |
| Tempo di riaggancio, mediano | **4.8 s** | 23.2 s |

Cinque volte più in fretta. E il contatto è reale: nei fotogrammi in cui il
bersaglio ricompare, le sue coordinate immagine valgono (-0.82, -0.89) e
(+0.73, -0.95), cioè gli **angoli** dell'inquadratura, a 78 e 65 m di distanza.
La geometria torna senza ipotesi aggiuntive: l'angolo del fotogramma cade a
√(50² + 37.5²) = 62.5 m, e ai due istanti il velivolo è inclinato di 0.26 e
0.32 rad, che spostano l'impronta di 50·tan(0.32) ≈ 17 m.

### Il difetto che la ricerca direzionale ha portato alla luce

Arrivare vicino al bersaglio ha però reso osservabile un difetto che prima non
aveva occasione di manifestarsi, e che rendeva il successo apparente.
Separando gli agganci iniziali da quelli riconquistati dopo una ricerca:

| | durata mediana | fotogrammi con bersaglio visto |
|---|---|---|
| Ricerca direzionale, aggancio iniziale | 105 s | 64.7% |
| Ricerca direzionale, dopo una ricerca | **4.4 s** | **0.0%** |
| Spirale, dopo una ricerca | 24.2 s | 64.8% |

Tutti e tre i riagganci del braccio direzionale erano ciechi: il rilevatore non
vedeva il bersaglio in nessun campione, e l'aggancio si sfaldava in quattro
secondi. Il meccanismo: il bersaglio compare per uno o due fotogrammi
nell'angolo dell'inquadratura, quel lampo resuscita il filtro, che da quel
momento pubblica predizioni valide per `soglia_perdita` fotogrammi; la missione
ne contava due e dichiarava il riaggancio. Per tutta la durata del finto
aggancio la missione **smetteva di cercare**, affidandosi a un controllo visivo
senza immagine: il finto riaggancio costava più di quanto valesse.

Il difetto non era nella ricerca ma nel criterio che la interrompe. Ora
l'ingresso in `AGGANCIO` richiede rilevamenti veri e consecutivi, contati sul
flusso a valle del jammer; restare in `AGGANCIO` può invece poggiare sulla
predizione. L'asimmetria è voluta: la predizione serve a superare le
micro-interruzioni di un inseguimento in corso, non a crearne uno.

| A parità di tutto il resto | criterio vecchio | criterio corretto |
|---|---|---|
| Fotogrammi con bersaglio visto in `AGGANCIO` | 64.7% | **70.1%** |
| Distanza mediana in `AGGANCIO` | 41.7 m | **24.2 m** |
| Durata mediana dell'aggancio | 19.4 s | **47.3 s** |
| Riagganci dichiarati | 75% | 33% |
| Agganci riconquistati | 3, da 4-9 s, visto 0.0% | 1, da 20.2 s, visto 54.9% |

I finti agganci sono spariti. Il calo dei riagganci dal 75% al 33% è il prezzo
apparente di quel 75%, che contava tre successi che non erano successi;
migliorano invece le grandezze che non dipendono dal conteggio, perché non ci
sono più episodi da quattro secondi a inquinare le mediane.

### Il secondo difetto: una stima di velocità non validata

Con il criterio corretto è emerso il limite vero. In una prova la ricerca è
durata 79.8 s partendo da 151.7 m con distanza minima **esattamente** quella
iniziale, e il log dice perché:

```
Bersaglio perso — ricerca da (205, 177) con velocita (-7.7, -12.8) m/s
RICERCA inseguimento cieco -> (179, 134) ... -> (141, 70)
```

Il bersaglio vero era a (340, 250) e si allontanava verso nord-est. Il drone ha
volato verso sud-ovest.

La causa non è un errore di segno. Nei fotogrammi che precedono ogni perdita la
posizione del bersaglio nell'immagine oscilla da un bordo all'altro — il
velivolo manovra e il bersaglio è al margine — e la velocità istantanea del
filtro fotografa quell'oscillazione. È la stessa debolezza già misurata quando
il termine di anticipo è stato provato, dove la stima correla 0.6 con il vero;
lì costava rumore nel comando, qui costa la direzione della ricerca.

Il dato che ha trasformato l'ipotesi in diagnosi sono le quattordici stime
registrate in una giornata di prove: tre valevano **41, 33 e 76 m/s**, contro un
bersaglio che non può superare i 15 imposti dal simulatore. Non stime rumorose:
stime **impossibili**, accettate senza controllo. Settantasei metri al secondo
estrapolati per otto secondi mandano il punto di ricerca a seicento metri dal
vero.

Quattro presidi, nessuno dei quali una taratura — e il quarto esiste perché il terzo, da solo, peggiorava le cose:

- **mediana su una finestra recente** (`finestra_velocita_s`) invece di un
  singolo campione: una direzione che si inverte fra un fotogramma e il
  successivo non sopravvive alla mediana, un moto vero sì;
- **limite fisico** (`vel_bersaglio_max`, 25 m/s cioè 90 km/h per un veicolo
  terrestre): una stima che lo supera viene **rifiutata**, non troncata. Un
  valore impossibile non significa «circa quello», significa «non lo so», e
  chi lo usa deve poterlo sapere. Senza velocità ricostruibile la ricerca non
  estrapola: vola all'ultima posizione nota e apre la spirale lì, che è la
  degradazione controllata;
- **la velocità predetta non è una misura.** Il filtro pubblicava come valida
  anche la velocità durante la predizione, dove lo stato resta congelato
  all'ultimo valore: la finestra si riempiva di copie dello stesso numero
  proprio nei fotogrammi che precedono la perdita, e la mediana di quindici
  copie di un valore è quel valore. La posizione predetta resta valida — è
  quella che tollera le micro-interruzioni — la velocità no;
- **ma una velocità stabilita da misure vere non diventa ignota appena le
  misure cessano.** Il presidio precedente, da solo, ha prodotto l'effetto
  opposto a quello voluto, e la prima prova lo ha mostrato subito: escluse le
  predizioni, negli ultimi fotogrammi entrano solo campioni di misure vere, la
  finestra da un secondo scende sotto il minimo di cinque — il rilevatore in
  fuga cala fino a 10 Hz, e 0.4 s a 10 Hz sono quattro campioni — e la velocità
  pubblicata alla perdita valeva `(+0.0, +0.0)`. La ricerca smetteva di
  estrapolare e volava all'ultima posizione nota, cioè la degradazione
  controllata invece della funzione. Lo sbaglio non era escludere le
  predizioni: era dedurne che la velocità fosse sconosciuta, quando era
  conosciuta mezzo secondo prima. Ora l'ultimo valore ben stabilito resta
  utilizzabile per `validita_velocita_s`, dopo il quale torna ignoto davvero.
  Il valore ricordato non viene mai sostituito da una stima fuori dal limite
  fisico: una misura impossibile non deve poter scacciare una buona.

**Quanto vale davvero quella stima.** Il sospetto naturale, davanti a mediane
stabilmente sopra il limite fisico, è un errore sistematico: se la velocità del
velivolo fosse sommata con il segno sbagliato si otterrebbe la somma dei moduli
invece della differenza. La verifica contro la verità a terra lo esclude —
l'errore mediano vale 9.3 m/s con la somma e 31.3 m/s con la differenza, quindi
il segno è quello giusto — ma nel farlo produce il numero che decide l'intera
questione.

Su **18 prove e 5426 campioni**, l'errore mediano della velocità ricostruita
vale **10.8 m/s**, contro un bersaglio che viaggia a **10.0 m/s**. Ogni singola
prova concorda, da 7.4 a 20.8 m/s. L'errore è grande quanto il segnale.

Da una grandezza così non si ricava una direzione, e nessun filtraggio a valle
può cambiarlo: mediana, limite fisico e memoria impediscono il disastro, non
creano l'informazione che manca. È il motivo per cui
`durata_inseguimento_cieco_s` è spento per default, con la stessa logica con cui
`k_anticipo` vale zero — misurato, non conveniente, lasciato parametrico con la
ragione scritta accanto. Migliorata la stima, si riaccende senza altre
modifiche.

L'effetto del limite fisico è comunque categorico prima che statistico:
**nessuna ricerca vola più nella direzione sbagliata.** Con il limite attivo tutte e
quattro le ricerche hanno chiuso distanza — 25, 83, 0 e 49 m recuperati —
mentre senza, una era andata da 151.7 a 151.7 m con il drone a 250 m dalla
parte opposta.

| A parità di tutto il resto | senza limite | con limite |
|---|---|---|
| Riagganci | 33% | 50% |
| Tempo di riaggancio, mediano | 13.8 s | 5.3 s |
| Distanza minima in ricerca | 81.8 m | 64.2 m |
| Ricerche che si sono avvicinate | 2 su 3 | **4 su 4** |

### Quanto di tutto questo è dimostrato

Le differenze sulle grandezze d'esito — percentuali di riaggancio, tempo in
`AGGANCIO`, durata degli agganci — sono **dentro la dispersione** misurata a
questa scala e non vanno lette come risultati: con tre prove per configurazione
e un fattore due-quattro fra prove identiche, tabelle di questo tipo mostrano
tendenze e non misure. Si veda il paragrafo sulla ripetibilità.

Ciò che è dimostrato è di natura diversa, e non richiede statistica:

- la spirale isotropa **non chiudeva la distanza**, con la minima uguale
  all'iniziale in tre prove su cinque;
- i riagganci del criterio vecchio erano ciechi, con **zero** fotogrammi
  rilevati su tre episodi, e ora non lo sono;
- tre stime di velocità su quattordici erano **fisicamente impossibili**, e ora
  vengono rifiutate;
- con il limite attivo **nessuna** ricerca si è allontanata dal bersaglio;
- l'errore della velocità stimata **vale quanto la velocità stessa** — 10.8
  contro 10.0 m/s su 5426 campioni — ed è la misura che, sola fra tutte quelle
  di questa sezione, non risente della dispersione.

Resta aperto il limite di fondo, che è quello già dichiarato da questo
progetto: la qualità della stima di velocità. Che sia profondo lo dicono due
osservazioni indipendenti — perfino la mediana su un secondo, venticinque
campioni, produceva ancora valori oltre il limite fisico, quindi il rumore è
correlato su tempi più lunghi della finestra e non si elimina filtrando; e
l'errore mediano eguaglia il segnale su 5426 campioni.

La via indicata dalla struttura del problema resta la stessa: fornire al filtro
la velocità del velivolo come ingresso noto, così che stimi direttamente la
velocità assoluta del bersaglio invece di ricavarla per differenza da
un'immagine in cui i due moti sono sovrapposti. È la stessa direzione già
indicata dalla prova sul termine di anticipo, e questa sezione la rafforza: due
funzioni diverse — guida predittiva e ricerca direzionale — si sono fermate
davanti allo stesso ostacolo, il che è un buon argomento perché sia quello
l'ostacolo da rimuovere.

**Cosa resta acceso.** Il centro della ricerca sull'ultima posizione nota del
bersaglio, che non dipende dalla velocità e la cui utilità è visibile nel
confronto con le prove storiche; il criterio di riaggancio sui rilevamenti veri;
i quattro presidi sulla stima di velocità, che valgono anche per chiunque la
usi in futuro. Spenta resta solo l'estrapolazione, in attesa della stima che la
renderebbe sensata.

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
metriche.py ricerche /ws/metrics/*_cieco8_*.csv
metriche.py gruppi '*_cieco8_*.csv' '*_spirale_*.csv'
```

**Figure del filtro.** `grafici.py` rigenera dalle stesse tracce le due figure
sul filtro di Kalman — stato stimato con `R` nel tempo, ed errore di posizione
con e senza filtro:

```bash
python3 /usr/local/share/drone_tracking/scripts/grafici.py \
    /ws/metrics/<prova>.csv /ws/metrics/figure
```

Esiste come script versionato e non come sessione interattiva per la stessa
ragione di `metriche.py`: una figura che finisce in una relazione deve poter
essere rifatta da chiunque, dallo stesso dato, ottenendo la stessa immagine.
Restringe da sé la finestra al tratto di `AGGANCIO` continuo più lungo — il
regime in cui il filtro lavora — così l'inquadratura non è scelta a occhio.

`riassumi` dà durata degli agganci, distanza mediana e media, frazione di
campioni con bersaglio, tempo per fase, ritmo della percezione. `confronta`
verifica la ripetibilità di due prove gemelle.

`ricerche` elenca ogni episodio di `RICERCA` con il suo esito: durata, distanza
al momento della perdita, distanza minima raggiunta, e se il bersaglio è stato
davvero ritrovato. Quest'ultimo punto non si legge dalla macchina a stati, che
dichiara il riaggancio su fotogrammi validi del tracker e quindi anche su
predizioni; la prova sta invece nella semantica del filtro, che dopo
`soglia_perdita` si azzera e può tornare valido solo ricevendo una misura vera.
Una risalita di `trk_valido` da 0 a 1 è percio un rilevamento avvenuto, anche
quando il campionamento a 5 Hz non lo vede — e spesso non lo vede, perché il
rilevatore pubblica a ~25 Hz e un avvistamento di un solo fotogramma ha una
probabilità su cinque di finire in un campione.

`gruppi` confronta due configurazioni con più prove ciascuna. Esiste per non
invitare più a usare `confronta` dove non si può: si veda il paragrafo seguente.

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

**Alla scala reale la ripetibilità è molto peggiore, e va detto.** Il 3% qui
sopra è stato misurato con il bersaglio a passo d'uomo su un'orbita da 3 m: a
quella scala il drone stava sopra il bersaglio per quasi tutta la prova e non
c'era molto che potesse andare diversamente. Con la fuga a 15 m/s la dinamica
diventa marginale — il drone recupera 5-10 m/s su un bersaglio che ne fa 15 — e
in una situazione marginale piccole differenze di istante decidono l'esito.

La misura, sei prove del 6 settembre 2026 a 120 s simulati, tre per
configurazione:

| | durate dell'aggancio iniziale |
|---|---|
| Configurazione A | 29 s, 107 s, 105 s |
| Configurazione B | 54 s, 74 s, 3 s |

Fino alla prima perdita del bersaglio le due configurazioni eseguono **codice
identico** — differiscono solo in cosa fanno durante `RICERCA` — quindi quei sei
numeri misurano la dispersione e nient'altro. Va da 3 a 107 secondi: un fattore
trenta fra il caso peggiore e il migliore, e un fattore due-quattro fra prove
della stessa configurazione.

Le conseguenze sul metodo sono due, e valgono per qualunque misura futura a
questa scala:

- **niente conclusioni da una prova sola**, e nemmeno da due: servono almeno
  tre ripetizioni per configurazione, alternate fra loro perché una deriva della
  macchina non cada tutta su una;
- **l'unità di analisi è l'episodio, non la prova**, dove la domanda lo
  consente. Per giudicare la ricerca, per esempio, la domanda è «dato che il
  bersaglio è stato perso, viene ritrovato?»: una prova che non lo perde mai non
  ha voce in capitolo, ma peserebbe eccome su una mediana per prova. Aggregando
  gli episodi il campione utile passa da tre valori a una decina. È la ragione
  per cui `metriche.py gruppi` stampa due blocchi separati.

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

La quota di decollo dev'essere **50 m**, coerente con i waypoint. La quota entra
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
| Espansione della spirale | 0.002 per chiamata a 2 Hz = **4 mm/s** | `ricerca_vel_espansione = 3.0` m/s |

**La spirale di ricerca non si allargava** — è il caso più estremo dello stesso
errore. `ricerca_espansione += 0.002` a ogni chiamata, su un timer a 2 Hz, dà
**4 millimetri al secondo**: per passare da 3 a 25 m di raggio servivano
**92 minuti**. In pratica non era una spirale ma un cerchio fisso di raggio 3 m,
mentre il bersaglio in fuga si allontanava a 1.2 m/s. Il drone entrava in
`RICERCA` e non ritrovava più nulla.

Corretta l'espansione, la spirale ha poi dovuto seguire la scala dello
scenario. Ai valori attuali — espansione **3.0 m/s**, velocità angolare
**0.25 rad/s** — un giro dura ~25 s e allarga il raggio di ~75 m, meno dei 100 m
di lato dell'impronta a 50 m di quota, quindi non restano porzioni di terreno
non guardate. Da `ricerca_raggio` = 30 m a `ricerca_raggio_max` = 300 m in 90
secondi, oltre i quali la ricerca è dichiarata fallita e la missione torna a
`PATTUGLIAMENTO` invece di allargarsi indefinitamente allontanandosi dall'area
di interesse.

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
interamente comandato da `target_mover_node` e la fisica non lo tocca. La quota
del modello vale metà della sua altezza — 0.8 m per il parallelepipedo attuale,
0.3 m per la sfera di allora — così poggia a terra invece di restare sospeso.

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

Il primo deve restituire `1` (bersaglio statico: se non lo è, la fisica lo
sposta), il
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
