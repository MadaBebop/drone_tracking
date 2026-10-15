# Installare il progetto su una Linux nuova

Il README spiega come **far partire** il sistema. Questo documento spiega come
**installarlo** da zero, cioè cosa deve esistere sulla macchina prima che quei
comandi abbiano senso.

Serve quando si ricostruisce una macchina virtuale, si passa a un portatile
nuovo, o qualcun altro deve riprodurre le misure. È stato scritto ripercorrendo
una ricostruzione vera, quindi contiene anche gli inciampi, che sono la parte
che fa perdere il pomeriggio.

Le versioni non sono negoziabili fra loro: ROS 2 Jazzy esiste solo per Ubuntu
24.04, il plugin `ardupilot_gazebo` compila contro `gz-sim8` di Gazebo
Harmonic, e ArduPilot è fissato al tag `Copter-4.7.0`. La ricetta verificata è
`docker/Dockerfile`: se qui e lì divergono, ha ragione il Dockerfile, perché
quello viene eseguito a ogni build.

## 0 · La macchina

**Ubuntu 24.04 LTS.** Non è una preferenza: su 22.04 o su una release più
recente ROS 2 Jazzy non si installa.

Su una macchina virtuale, come riferimento su un host con 8 core fisici e 32 GB:

| Voce | Valore | Perché |
|---|---|---|
| Processori | **1 socket × 8 core** | Un socket, non otto: con più socket Linux assume distanze NUMA inesistenti e sparpaglia i processi fra nodi immaginari |
| Memoria | **12 GB** | Gazebo, SITL, MAVProxy e sette nodi ROS stanno stretti sotto gli 8 |
| Disco | **60 GB** | La sola compilazione di ArduPilot con i sottomoduli ne occupa parecchi |
| Virtualize VT-x/EPT | **spento** | Serve solo per annidare un hypervisor. I container non sono macchine virtuali e non lo richiedono |
| Accelerazione 3D | **spenta** | Quella di VMware con Gazebo dà artefatti; il progetto usa già `LIBGL_ALWAYS_SOFTWARE=1` |

La simulazione gira comunque a **0,24-0,32 volte il tempo reale**: due minuti
simulati ne costano sei o sette veri. È il motivo per cui le batterie di prove
si lanciano staccate e si guardano dopo.

## 1 · ROS 2 Jazzy

```bash
sudo apt update && sudo apt install -y software-properties-common curl git
sudo add-apt-repository -y universe
```

```bash
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo apt install -y /tmp/ros2-apt-source.deb
sudo apt update && sudo apt install -y ros-jazzy-ros-base python3-colcon-common-extensions
```

> Il meccanismo del repository ROS è cambiato più volte negli anni. Se questi
> comandi falliscono, la fonte autorevole è
> `docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html`; tutto il resto
> di questo documento resta valido.

## 2 · Gazebo Harmonic

```bash
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt update && sudo apt install -y gz-harmonic
```

## 3 · Lo stack ROS del progetto

```bash
sudo apt install -y \
  ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-mavros-msgs \
  ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image \
  ros-jazzy-cv-bridge ros-jazzy-vision-opencv \
  ros-jazzy-rqt-image-view ros-jazzy-rqt-plot ros-jazzy-rqt-graph \
  python3-opencv python3-numpy geographiclib-tools
```

```bash
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
```

Quest'ultimo non è opzionale: senza i dataset dei geoidi **MAVROS muore
all'avvio**, e il messaggio d'errore non dice che mancano quelli.

## 4 · ArduPilot SITL

```bash
sudo apt install -y build-essential ccache g++ cmake pkg-config \
  rapidjson-dev libopencv-dev python3-dev python3-venv python3-pip
cd ~ && git clone --recurse-submodules --depth 1 --branch Copter-4.7.0 \
  https://github.com/ArduPilot/ardupilot.git
```

Le dipendenze di compilazione vanno in un ambiente virtuale: Ubuntu 24.04 è
*externally managed* (PEP 668) e `pip` rifiuta di installare a livello di
sistema.

```bash
python3 -m venv ~/venv-ardupilot
~/venv-ardupilot/bin/pip install empy==3.3.4 pexpect future lxml pymavlink setuptools
cd ~/ardupilot
PATH="$HOME/venv-ardupilot/bin:$PATH" ./waf configure --board sitl
PATH="$HOME/venv-ardupilot/bin:$PATH" ./waf copter
```

Circa venti minuti. **Attenzione al tag**: i branch di release si fermano a
Copter-4.5, le versioni successive esistono solo come tag, e `master` si
dichiara 4.8.0-dev ma è un bersaglio mobile.

## 5 · Il plugin `ardupilot_gazebo`

```bash
sudo apt install -y libgz-sim8-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  cppzmq-dev libzmq3-dev
```

Le ultime quattro sembrano superflue e non lo sono: il `CMakeLists` cerca
GStreamer come **required** per un plugin camera che a noi non serve ma che
blocca comunque la configurazione dell'intero progetto, e `gz-transport13`
espone `CPPZMQ::CPPZMQ` nella propria interfaccia di link, quindi il target
CMake deve esistere anche se è solo header.

```bash
cd ~ && git clone https://github.com/ArduPilot/ardupilot_gazebo.git
mkdir -p ~/ardupilot_gazebo/build && cd ~/ardupilot_gazebo/build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j"$(nproc)"
```

## 6 · MAVProxy, con la console

```bash
sudo apt install -y python3-wxgtk4.0
python3 -m venv --system-site-packages ~/venv-mavproxy
~/venv-mavproxy/bin/pip install MAVProxy pymavlink future pyserial lxml
sudo ln -sf ~/venv-mavproxy/bin/mavproxy.py /usr/local/bin/mavproxy.py
```

Tre dettagli, tutti costati tempo.

`--system-site-packages` è ciò che fa vedere al venv il wxPython installato da
apt: senza, il flag `--console` non apre nulla e non spiega perché. Il
container non ha wxPython di proposito, perché lì non c'è GUI, quindi chi copia la
ricetta dal `Dockerfile` resta senza console.

`future` non è dichiarato dalla wheel di MAVProxy ma è importato da
`modules/lib/rline.py`: senza, `mavproxy.py` muore all'avvio.

**Il venv non va messo nel PATH.** Mascherebbe il `python3` di sistema e con
lui `rclpy`, `numpy` e `cv2`. Si espone solo l'eseguibile con un symlink: il
suo shebang punta già all'interprete giusto.

## 7 · Il workspace

```bash
git clone <url-del-repository> ~/drone_tracking_ws
cd ~/drone_tracking_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## 8 · Collegare gli asset di simulazione

Il repository versiona mondo e modello sotto `sim/`, ma `gz sim` li carica da
`ardupilot_gazebo/`. Senza questo passaggio `git pull` aggiorna file che la
simulazione non legge mai, e il guasto è silenzioso, perché tutto continua a
funzionare mostrando la versione vecchia.

```bash
cd ~/ardupilot_gazebo/worlds
mv iris_runway.sdf iris_runway.sdf.bak
ln -s ~/drone_tracking_ws/sim/worlds/iris_runway.sdf .

cd ~/ardupilot_gazebo/models/iris_with_ardupilot
mv model.sdf model.sdf.bak
ln -s ~/drone_tracking_ws/sim/models/iris_with_ardupilot/model.sdf .
```

Il bersaglio non è un modello a sé: è definito dentro il file del mondo, quindi
i collegamenti da rifare sono due e non tre.

## 9 · L'ambiente

```bash
cat >> ~/.bashrc <<'EOF'
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/drone_tracking_ws/install/setup.bash" ] && source "$HOME/drone_tracking_ws/install/setup.bash"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build"
export GZ_SIM_RESOURCE_PATH="$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds"
export GZ_CONFIG_PATH="/usr/share/gz${GZ_CONFIG_PATH:+:$GZ_CONFIG_PATH}"
export LIBGL_ALWAYS_SOFTWARE=1
export ROS_DOMAIN_ID=42
EOF
exec bash
```

**La riga su `GZ_CONFIG_PATH` deve stare dopo quella del `source`, e non è
facoltativa.** Il `setup.bash` di ROS 2 sovrascrive quella variabile con i soli
percorsi dei pacchetti vendored, dove il sottocomando `sim` non esiste: da quel
momento `gz sim` stampa l'elenco dei comandi disponibili e **termina con esito
zero**, come se tutto fosse a posto. Gazebo è installato: è il comando che non
viene più trovato. Le *Note tecniche* del README raccontano la stessa trappola
nella sua forma dentro il container.

Verifica che sia risolta:

```bash
gz sim --versions      # deve rispondere 8.x.x
```

## 10 · Avvio: cinque terminali, in quest'ordine

L'ordine è obbligatorio: il SITL deve trovare Gazebo già in ascolto sulle porte
JSON.

```bash
# T1. Gazebo (senza -s, così si apre la finestra 3D)
gz sim -v4 -r ~/ardupilot_gazebo/worlds/iris_runway.sdf
```

```bash
# T2. ArduPilot SITL
cd ~/ardupilot && ./build/sitl/bin/arducopter --model JSON --speedup 1 \
  --sim-address=127.0.0.1 --sim-port-in=9013 --sim-port-out=9012 \
  -I0 --defaults Tools/autotest/default_params/gazebo-iris.parm,$HOME/drone_tracking_ws/docker/sitl-defaults.parm
```

```bash
# T3. MAVProxy, con la finestra di stato
mavproxy.py --master tcp:127.0.0.1:5760 \
  --out 127.0.0.1:14550 --out udp:127.0.0.1:14555 --console
```

```bash
# T4. MAVROS
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://:14555@127.0.0.1:14556 \
  -p target_system_id:=1 -p target_component_id:=1 \
  -p plugin_denylist:=[distance_sensor]
```

```bash
# T5. I nodi del progetto
cd ~/drone_tracking_ws && source install/setup.bash
ros2 launch drone_tracking tracking.launch.py
```

Poi, nella console di MAVProxy: `mode guided`, `arm throttle`, `takeoff 50`. E
infine `ros2 topic pub --once /mission/avvia std_msgs/msg/Bool "data: true"`.

**MAVROS è un processo a sé: il launch file non lo avvia.** Dimenticarlo è il
modo più rapido di perdere un'ora: i sette nodi partono, `ros2 node list` li
elenca tutti, e `mission_node` non vede mai la posizione, quindi la distanza
dal waypoint resta infinita e il drone non si muove.

E va collegato all'**uscita UDP di MAVProxy**, non alla porta TCP del SITL: su
`SERIAL1` ArduPilot manda l'heartbeat ma non gli stream di posizione, quindi
`/mavros/state` funziona mentre `local_position/pose` resta muto, ed è il guasto
peggiore, perché sembra che la connessione sia a posto.

## 11 · Guardare cosa succede

```bash
ros2 run rqt_image_view rqt_image_view /target/debug_image   # cosa vede il rilevatore
ros2 run rqt_image_view rqt_image_view /drone/camera/image_raw
~/drone_tracking_ws/scripts/monitor.sh                       # cruscotto della missione
```

`/target/debug_image` **non viene pubblicato finché la missione è in `ATTESA`**:
il rilevatore esce prima. Se apri la finestra prima di avviare la missione vedi
«No image» e concludi che la telecamera sia rotta. Per verificare il ponte
prima del decollo si guarda `image_raw`, che Gazebo pubblica sempre.

## 12 · L'alternativa: Docker dentro la stessa VM

I passi 1-9 si sostituiscono con:

```bash
sudo apt update && sudo apt install -y docker.io docker-compose-v2 git
sudo usermod -aG docker $USER        # richiede logout e login
git clone <url-del-repository> ~/drone_tracking_ws && cd ~/drone_tracking_ws
docker compose up -d --build
docker compose exec sim start_all.sh
```

Una trentina di minuti non presidiati invece di un pomeriggio, e si ottengono
esattamente le versioni con cui il progetto è stato misurato. Per la GUI di
Gazebo basta mettere `HEADLESS: "0"` e togliere il commento alle righe
`DISPLAY` nel `docker-compose.yml`: su Linux funziona, a differenza di Windows.

**Dopo ogni `docker compose up -d` che ricrea il container serve ricostruire il
workspace**, perché `/ws/install` non è un volume e torna allo stato
dell'immagine: i nodi aggiunti dopo l'ultima build spariscono, il launch non
trova l'eseguibile e metà stack non parte, senza che nulla lo dica.

```bash
docker exec drone-tracking bash -lc \
  'source /opt/ros/jazzy/setup.bash && cd /ws && colcon build --symlink-install'
```

## Due messaggi che mentono

**`gz sim` che stampa l'elenco dei comandi** non vuol dire che Gazebo manchi:
vuol dire che `GZ_CONFIG_PATH` è stato sovrascritto. Vedi il passo 9.

**`Arming checks disabled`, annunciato da MAVProxy a ogni armamento, è falso**
anche quando i controlli sono tutti attivi, come lo sono in questo progetto
(`ARMING_SKIPCHK 0`). Il modulo `arm` di MAVProxy legge `ARMING_CHECK`, che in
questa versione di ArduPilot non esiste, ottiene `None` e conclude che siano
spenti. Dopo un avvio a freddo il primo tentativo di arming può essere
rifiutato con `Arm: Gyros inconsistent`: si ritenta e passa, e `takeoff.sh` lo
fa da solo fino a cinque volte.
