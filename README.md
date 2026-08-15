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

**Percorso di pattugliamento** — circuito quadrato a 12 m di quota, con soglia di
raggiungimento a 1.2 m:

```
(0,0) → (20,0) → (20,20) → (0,20) → (0,0)
```

Il vertice `(20,20)` coincide con la zona in cui orbita il bersaglio. Se il giro
si chiude senza aggancio, il pattugliamento riparte dal waypoint 1.

**Aggancio** — richiede **5 frame consecutivi** con bersaglio visibile, e solo
sopra i 2 m di quota, per evitare falsi positivi durante il decollo.

**Perdita e ricerca** — dopo **20 frame** (~2 s) senza bersaglio in `AGGANCIO`,
la fase passa a `RICERCA`: il drone descrive una spirale attorno alla propria
posizione al momento della perdita, con raggio iniziale 3 m che cresce di
0.002 m per tick. Bastano 5 frame consecutivi di nuova visibilità per tornare in
`AGGANCIO`.

### detector_node

Legge il feed della telecamera montata sul drone (inclinata a 60° verso il basso,
FOV orizzontale 60°, 640×480 a 30 Hz) e isola il bersaglio rosso con doppia
soglia HSV — due intervalli, perché il rosso è a cavallo del wrap-around della
tinta:

```
mask1: H ∈ [0, 10]     S ∈ [120, 255]  V ∈ [70, 255]
mask2: H ∈ [170, 180]  S ∈ [120, 255]  V ∈ [70, 255]
```

Del contorno più grande calcola il centroide via momenti di immagine e lo
normalizza in `[-1, +1]`. I contorni sotto **200 px²** vengono scartati come
rumore visivo.

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
Cicla automaticamente ON/OFF: **40 tick attivo (~4 s)**, **100 tick inattivo
(~10 s)**.

Quando il jamming è attivo:

- pubblica `/gps/jammed: true` e `/rf/noise_level: 0.8`
- inietta rumore gaussiano (σ = 0.3) sulle coordinate, con clamp a `[-1, +1]`
- con probabilità **30%** simula la perdita totale del pacchetto (azzera x, y, z)

Due accorgimenti importanti: non corrompe segnali già nulli (non genera falsi
positivi dal nulla) ed è limitato a un messaggio ogni 50 ms, perché a piena
frequenza il tracker veniva sovraccaricato.

### tracker_node

Filtro di Kalman a 4 stati `[x, y, vx, vy]` in coordinate immagine.

| Matrice | Nome nel codice | Ruolo |
|---|---|---|
| F | `evoluzione_stato` | Modello cinematico a velocità costante (dt = 0.1) |
| H | `mappa_osservazione` | Osserva solo posizione x/y |
| Q | `incertezza_modello` | `diag(0.01, 0.01, 0.5, 0.5)` — alta su velocità |
| R | `incertezza_sensore` | Adattiva, vedi sotto |
| K | `guadagno_kalman` | Bilancia modello e misura |

**R adattiva** — è il punto centrale della resistenza al jamming. Il tracker si
iscrive a `/rf/noise_level` e ricalcola l'incertezza di misura a ogni variazione:

```
R = 0.05 + (2.0 − 0.05) · livello_rumore
```

Con il jamming attivo (`livello_rumore = 0.8`) l'incertezza di misura sale di
circa 30 volte: il guadagno di Kalman crolla e il filtro dà fiducia alla propria
predizione invece che ai dati corrotti. Senza jamming torna a fidarsi del
detector. È l'equivalente in coordinate immagine di un INS che degrada
gracefully quando il GNSS diventa inaffidabile.

**Gestione della perdita di segnale** — durante i buchi il filtro pubblica la
predizione, tollerando fino a **5 frame** consecutivi; oltre quella soglia si
resetta e pubblica un punto nullo, segnalando la perdita a mission e controller.

La velocità stimata viene inoltre smorzata al **60%** a ogni correzione: senza
questo freno, il rumore del jammer veniva interpretato come moto reale del
bersaglio e la predizione divergeva.

### controller_node

Controllo **proporzionale-derivativo** che traduce l'errore di posizione nel
frame della telecamera in comandi di velocità in body frame.

| Parametro | Valore |
|---|---|
| `kp_x`, `kp_y` | 4.0 (scalati con la quota) |
| `kd_x`, `kd_y` | 0.8 |
| `vel_max` | 8.0 m/s |
| `deadzone` | 0.05 |
| Frequenza | 10 Hz |

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

**Scalatura con la quota** — i guadagni sono moltiplicati per
`quota / 12.0`: lo stesso errore in pixel corrisponde a una distanza reale
maggiore quando il drone è alto, quindi il guadagno cresce con la quota e la
risposta resta coerente lungo tutto l'inseguimento.

**Guardia FOV** — le posizioni con `|x| > 1.2` o `|y| > 1.2` vengono ignorate:
sono predizioni di Kalman ormai fuori dal campo visivo, inseguirle porterebbe il
drone fuori strada.

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
| `PATTUGLIO` | Orbita circolare attorno a `(20, 20)`, raggio 3 m | 0.35 rad/s ≈ **1.05 m/s** |
| `EVASIONE` | Fuga in linea retta opposta al drone, per 20 s | **1.2 m/s** |

**Perché queste velocità.** Il limite non è la velocità massima del drone, che
arriva a 8 m/s, ma l'**errore a regime** del controllo proporzionale: inseguendo
un bersaglio a velocità costante, l'errore d'immagine si stabilizza intorno a
`velocità_bersaglio / kp`. Con `kp = 4.0`:

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
un tracking discontinuo il ritardo osservato è più lungo. Terminata la fuga di
`durata_evasione_s = 49.0` secondi riprende a orbitare attorno alla nuova
posizione.

Il comportamento osservato in simulazione è quello descritto sopra: è questo nodo
a muovere il bersaglio. Il modello nel world dichiara anche un plugin
`TrajectoryFollower` che però resta inerte — vedi *Problemi noti*.

---

## Avvio manuale (installazione nativa)

Da usare sulla VM/macchina Ubuntu dove ArduPilot e Gazebo sono installati
localmente. Sostituire i percorsi con i propri. L'ordine è obbligatorio.

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
  -I0 --defaults Tools/autotest/default_params/gazebo-iris.parm
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

L'ultimo parametro esclude il plugin `distance_sensor`, che altrimenti riempie i
log di `DS: no mapping for sensor id: 0, type: 4, orientation: 25` più volte al
secondo. Vedi *Problemi noti*. Omettendolo il sistema funziona lo stesso, ma i log
diventano illeggibili.

**T5 — Nodi ROS 2**

```bash
export RCUTILS_COLORIZED_OUTPUT=1
cd "$HOME/Desktop/Progetto Drone/drone_tracking_ws"
source install/setup.bash
ros2 launch drone_tracking tracking.launch.py
```

Il launch file avvia i sei nodi **e** il `parameter_bridge` di `ros_gz` che porta
`/drone/camera/image_raw` da Gazebo a ROS 2.

**T6 — Decollo** (nella console MAVProxy)

```
param set ARMING_CHECK 0
mode guided
arm throttle
takeoff 12
```

**T7 — Avvio missione**

```bash
ros2 topic pub --once /mission/avvia std_msgs/msg/Bool "data: true"
```

La quota di decollo dev'essere **12 m**, coerente con i waypoint e con
l'`altitudine_crociera` usata dal controller per scalare i guadagni.

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

**Telecamera** — link `camera_link` fissato a `base_link`, pose
`0.1 0 -0.05 0 1.047 0`: inclinata di **1.047 rad = 60°** verso il basso, non 45°
come indicato in una versione precedente di questo documento. Con FOV orizzontale
di 60° e quota di crociera 12 m, l'asse ottico tocca terra a ~14 m di distanza in
obliquo e l'inquadratura copre una quindicina di metri in larghezza.

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
| Soglia di avvio ricerca | 20 frame → fra 1.5 e 4 s secondo il carico | `soglia_avvia_ricerca_s = 2.0` |
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

**L'immagine sobbalza, ma il drone è fermo** — con il rendering software i frame
non arrivano a cadenza regolare, e a schermo l'effetto è un video che "salta".
Non è un'oscillazione del velivolo: la verità di Gazebo, campionata a 30 Hz dal
topic delle pose, dà in hover **roll std 0.26°** ed escursione ±0.4°, con pitch
praticamente nullo. Il drone è stabile.

Attenzione a non farsi ingannare da `/mavros/imu/data`: pubblica a ~1.6 Hz e
campionarlo suggerisce oscillazioni di 0.4 rad/s che non esistono — è aliasing.
Per giudicare l'assetto va usato il topic delle pose di Gazebo, non l'IMU via
MAVLink.

Il rimedio applicato è abbassare `<update_rate>` della telecamera da 30 a **15**:
chiedere una frequenza irraggiungibile faceva mancare al renderer ogni scadenza,
consegnando i frame quando capitava. Misure su 20 s:

| | `update_rate` 30 | `update_rate` 15 |
|---|---|---|
| Frequenza effettiva | 11.4 Hz | 10.9 Hz |
| Deviazione standard | 31.2 ms | **14.2 ms** |
| Intervallo peggiore | 397.6 ms | **179.4 ms** |
| Jitter relativo | 36.4% | **17.1%** |

Il jitter si dimezza senza perdere frequenza. Il residuo dipende dal
rasterizzatore software: sparisce con accelerazione grafica.

**Frame rate della telecamera nel container** — il sensore dichiara
`<update_rate>30</update_rate>`, ma in headless senza GPU Gazebo renderizza via
rasterizzatore software e il topic `/drone/camera/image_raw` resta molto sotto:
misurato fra **5 e 11 Hz** su WSL2, a seconda del carico della macchina. Il
tracking funziona comunque, ma
va tenuto presente che il filtro di Kalman assume `dt = 0.1` (10 Hz) nella matrice
`evoluzione_stato`: a 5 Hz il passo di predizione sottostima lo spostamento reale
fra un frame e l'altro, rendendo il tracker più lento a inseguire un bersaglio in
evasione. Per un confronto quantitativo con la VM, misurare prima il rate reale:

```bash
ros2 topic hz /drone/camera/image_raw
```

Se serve fedeltà, conviene allineare `dt` al rate misurato o eseguire la
simulazione con accelerazione grafica.

**Versione del firmware** — ArduPilot non pubblica più un branch per ogni
release: i branch si fermano a `Copter-4.5`, le versioni successive esistono solo
come **tag**. Un binario compilato da `master` si dichiara `4.8.0-dev` pur non
corrispondendo ad alcuna release: è da lì che veniva il "v4.8.0" indicato in una
versione precedente di questo documento. Il container usa il tag stabile
`Copter-4.7.0`, modificabile con l'argomento di build `ARDUPILOT_REF`.

**Prearm check** — il SITL con backend JSON fallisce spesso i controlli sui
sensori simulati. `ARMING_CHECK=0` prima dell'arming (già incluso in
`takeoff.sh`).

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
