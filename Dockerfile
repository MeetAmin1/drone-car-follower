FROM ros:jazzy-ros-base-noble

ARG DEBIAN_FRONTEND=noninteractive
ARG PX4_VERSION=v1.17.0
ARG MICRO_XRCE_AGENT_VERSION=v2.4.3

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git ninja-build python3-colcon-common-extensions python3-pip \
    python3-pytest python3-yaml python3-rosdep python3-vcstool python3-matplotlib \
    ros-jazzy-ros-gz ros-jazzy-nav-msgs curl wget ca-certificates jq \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --branch ${PX4_VERSION} --recursive https://github.com/PX4/PX4-Autopilot.git /opt/PX4-Autopilot \
    && RUNS_IN_DOCKER=true bash /opt/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx \
    && make -C /opt/PX4-Autopilot px4_sitl_default \
    && test -x /opt/PX4-Autopilot/build/px4_sitl_default/bin/px4 \
    && test -f /opt/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh \
    && test -f /opt/PX4-Autopilot/src/modules/simulation/gz_bridge/server.config

RUN git clone --depth 1 --branch ${MICRO_XRCE_AGENT_VERSION} https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/Micro-XRCE-DDS-Agent \
    && cmake -S /tmp/Micro-XRCE-DDS-Agent -B /tmp/Micro-XRCE-DDS-Agent/build \
    && cmake --build /tmp/Micro-XRCE-DDS-Agent/build -j"$(nproc)" \
    && cmake --install /tmp/Micro-XRCE-DDS-Agent/build \
    && ldconfig \
    && command -v MicroXRCEAgent >/dev/null \
    && rm -rf /tmp/Micro-XRCE-DDS-Agent

RUN python3 -m pip install --break-system-packages --no-cache-dir pymavlink

WORKDIR /workspace
RUN mkdir -p /workspace/src \
    && git clone --depth 1 --branch v1.17.0 https://github.com/PX4/px4_msgs.git /workspace/src/px4_msgs
COPY . /workspace/src/drone-car-follower

RUN source /opt/ros/jazzy/setup.bash \
    && if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then rosdep init; fi \
    && rosdep update \
    && rosdep install --from-paths /workspace/src --ignore-src -r -y \
    && colcon build --symlink-install \
    && test -f /workspace/install/setup.bash \
    && pytest -q /workspace/src/drone-car-follower/tests

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENV PX4_AUTOPILOT_DIR=/opt/PX4-Autopilot
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "drone_system", "full_stack.launch.py"]
