# syntax=docker/dockerfile:1
#
# Drone Tracking — immagine all-in-one
#   ROS 2 Jazzy + Gazebo Harmonic + ArduPilot SITL + ardupilot_gazebo + MAVROS2
#
# Build multi-stage: lo stage "builder" compila ArduPilot e il plugin Gazebo,
# lo stage finale copia solo gli artefatti (l'immagine finale pesa ~4 GB invece di ~12).
#
#   docker compose build
#   docker compose up -d && docker compose exec sim start_all.sh
#

# =============================================================================
# Stage 1 — builder
# =============================================================================
FROM ros:jazzy-ros-base AS builder

ARG DEBIAN_FRONTEND=noninteractive

# Tag di ArduPilot da compilare. Attenzione: i branch di release si fermano a
# Copter-4.5, le versioni successive esistono solo come tag. `master` si dichiara
# 4.8.0-dev ma è un bersaglio mobile: qui si usa l'ultimo tag stabile.
ARG ARDUPILOT_REF=Copter-4.7.0
# Branch di ardupilot_gazebo compatibile con Gazebo Harmonic (gz-sim8).
ARG ARDUPILOT_GAZEBO_REF=main

RUN apt-get update && apt-get install -y --no-install-recommends \
        git ca-certificates curl gnupg lsb-release \
        build-essential ccache g++ cmake pkg-config \
        rapidjson-dev libopencv-dev \
        python3-dev python3-venv python3-pip \
    && rm -rf /var/lib/apt/lists/*

# --- Repository OSRF: serve libgz-sim8-dev per compilare il plugin ArduPilot ---
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        libgz-sim8-dev \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# ArduPilot SITL
# -----------------------------------------------------------------------------
WORKDIR /opt
RUN git clone --recurse-submodules --depth 1 --branch ${ARDUPILOT_REF} \
        https://github.com/ArduPilot/ardupilot.git

WORKDIR /opt/ardupilot
# Le dipendenze di build di ArduPilot vivono in un venv: Ubuntu 24.04 è
# externally-managed (PEP 668) e pip rifiuta di installare a livello di sistema.
RUN python3 -m venv /opt/venv-ardupilot \
    && /opt/venv-ardupilot/bin/pip install --no-cache-dir \
        empy==3.3.4 pexpect future lxml pymavlink setuptools

ENV PATH="/opt/venv-ardupilot/bin:${PATH}"
RUN ./waf configure --board sitl \
    && ./waf copter \
    && strip build/sitl/bin/arducopter

# -----------------------------------------------------------------------------
# ardupilot_gazebo (plugin + modelli + mondi upstream)
# -----------------------------------------------------------------------------
# Dipendenze di configure che libgz-sim8-dev non tira dietro da solo:
#  - GStreamer: cercato come REQUIRED dal CMakeLists per GstCameraPlugin, che a
#    noi non serve ma blocca comunque il configure dell'intero progetto.
#  - cppzmq: gz-transport13 espone CPPZMQ::CPPZMQ nel proprio link interface,
#    quindi il target CMake deve esistere anche se è header-only.
# Layer separato di proposito: aggiungerli al primo apt invaliderebbe la cache
# della compilazione di ArduPilot, che dura ~20 minuti.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
        cppzmq-dev libzmq3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt
RUN git clone --depth 1 --branch ${ARDUPILOT_GAZEBO_REF} \
        https://github.com/ArduPilot/ardupilot_gazebo.git

WORKDIR /opt/ardupilot_gazebo/build
RUN cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && make -j"$(nproc)"


# =============================================================================
# Stage 2 — runtime
# =============================================================================
FROM ros:jazzy-ros-base AS runtime

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg ca-certificates lsb-release \
        python3-venv python3-pip \
        tmux nano less iproute2 iputils-ping net-tools \
        geographiclib-tools \
    && rm -rf /var/lib/apt/lists/*

# --- Gazebo Harmonic (CLI `gz sim` + runtime) ---
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# --- Stack ROS 2: MAVROS, ponte ros_gz, visione, tool di monitoraggio ---
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-mavros-msgs \
        ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image \
        ros-jazzy-cv-bridge ros-jazzy-vision-opencv \
        ros-jazzy-rqt-image-view ros-jazzy-rqt-plot ros-jazzy-rqt-graph \
        python3-opencv python3-numpy \
        python3-colcon-common-extensions \
        libgstreamer1.0-0 libgstreamer-plugins-base1.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Dataset geoidi richiesti da MAVROS all'avvio (altrimenti il nodo muore).
RUN /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh

# --- MAVProxy in venv dedicato (niente wxPython: la console GUI non serve) ---
# `future` non è dichiarato dalla wheel di MAVProxy ma è importato da
# modules/lib/rline.py: senza, mavproxy.py muore all'avvio.
# Il venv NON va messo nel PATH: maschererebbe il python3 di sistema e con esso
# rclpy, numpy e cv2. Si espone solo l'eseguibile, il cui shebang punta già
# all'interprete del venv.
RUN python3 -m venv /opt/venv-mavproxy \
    && /opt/venv-mavproxy/bin/pip install --no-cache-dir \
        MAVProxy pymavlink future pyserial lxml \
    && ln -sf /opt/venv-mavproxy/bin/mavproxy.py /usr/local/bin/mavproxy.py

# --- Artefatti dal builder ---
COPY --from=builder /opt/ardupilot/build/sitl/bin/arducopter /opt/ardupilot/build/sitl/bin/arducopter
COPY --from=builder /opt/ardupilot/Tools/autotest/default_params /opt/ardupilot/Tools/autotest/default_params
COPY --from=builder /opt/ardupilot_gazebo/build/ /opt/ardupilot_gazebo/build/
COPY --from=builder /opt/ardupilot_gazebo/models/ /opt/ardupilot_gazebo/models/
COPY --from=builder /opt/ardupilot_gazebo/worlds/ /opt/ardupilot_gazebo/worlds/

# --- Asset di simulazione custom del progetto (camera 45°, sfera bersaglio) ---
# Sovrascrivono i corrispettivi upstream. Se sim/ è vuota l'immagine resta valida
# e usa i mondi/modelli standard di ardupilot_gazebo.
COPY sim/ /opt/sim/
RUN if [ -d /opt/sim/models ] && [ -n "$(ls -A /opt/sim/models 2>/dev/null)" ]; then \
        cp -r /opt/sim/models/. /opt/ardupilot_gazebo/models/; fi \
    && if [ -d /opt/sim/worlds ] && [ -n "$(ls -A /opt/sim/worlds 2>/dev/null)" ]; then \
        cp -r /opt/sim/worlds/. /opt/ardupilot_gazebo/worlds/; fi

# --- Workspace ROS 2 del progetto ---
WORKDIR /ws
COPY src/drone_tracking/ /ws/src/drone_tracking/
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install \
    && rm -rf /ws/log

# --- Script di orchestrazione ---
COPY docker/entrypoint.sh     /usr/local/bin/entrypoint.sh
COPY docker/start_all.sh      /usr/local/bin/start_all.sh
COPY docker/sitl-defaults.parm /opt/sitl-defaults.parm
COPY scripts/            /usr/local/share/drone_tracking/scripts/
# I file arrivano da un host Windows: i CRLF romperebbero lo shebang.
RUN sed -i 's/\r$//' /usr/local/bin/entrypoint.sh /usr/local/bin/start_all.sh \
    && find /usr/local/share/drone_tracking/scripts -type f \
        \( -name '*.sh' -o -name '*.py' \) -exec sed -i 's/\r$//' {} + \
    && chmod +x /usr/local/bin/entrypoint.sh /usr/local/bin/start_all.sh \
    && find /usr/local/share/drone_tracking/scripts -type f \
        \( -name '*.sh' -o -name '*.py' \) -exec chmod +x {} + \
    && ln -sf /usr/local/share/drone_tracking/scripts/takeoff.sh  /usr/local/bin/takeoff.sh \
    && ln -sf /usr/local/share/drone_tracking/scripts/snapshot.sh /usr/local/bin/snapshot.sh \
    && ln -sf /usr/local/share/drone_tracking/scripts/monitor.sh  /usr/local/bin/monitor.sh \
    && ln -sf /usr/local/share/drone_tracking/scripts/metriche.py /usr/local/bin/metriche.py \
    && ln -sf /usr/local/share/drone_tracking/scripts/salva_config.sh /usr/local/bin/salva_config.sh \
    && ln -sf /usr/local/share/drone_tracking/scripts/prova.sh /usr/local/bin/prova.sh

# --- Shell interattive ---
# `docker exec` NON esegue l'ENTRYPOINT, quindi una shell aperta con
# `docker compose exec sim bash` non avrebbe ROS 2 nel PATH. Il source va nel
# .bashrc, che è l'unico punto letto in entrambi i casi.
# Le shell di login leggono /etc/profile.d/ e non .bashrc: si coprono entrambe.
RUN printf '%s\n' \
        'source /opt/ros/jazzy/setup.bash' \
        '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' \
        > /etc/profile.d/ros_env.sh \
    && cat /etc/profile.d/ros_env.sh >> /root/.bashrc

# --- Ambiente ---
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_gazebo/build \
    GZ_SIM_RESOURCE_PATH=/opt/ardupilot_gazebo/models:/opt/ardupilot_gazebo/worlds \
    ARDUPILOT_HOME=/opt/ardupilot \
    WORLD=iris_runway.sdf \
    HEADLESS=1 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    RCUTILS_COLORIZED_OUTPUT=1

EXPOSE 5760 14550/udp 14555/udp

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
