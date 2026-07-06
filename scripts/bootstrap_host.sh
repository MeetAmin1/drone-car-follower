#!/usr/bin/env bash
set -Eeuo pipefail

if [[ "${EUID}" -eq 0 ]]; then
  echo "Run this script as a normal user; it invokes sudo only for package installation." >&2
  exit 1
fi

if ! grep -q 'VERSION_ID="24.04"' /etc/os-release; then
  echo "This bootstrap is pinned to Ubuntu 24.04. Use the Docker path on other systems." >&2
  exit 1
fi

ROS_DISTRO=jazzy
ROS_APT_SOURCE_VERSION=1.2.0
PX4_VERSION=v1.17.0
MICRO_XRCE_AGENT_VERSION=v2.4.3
PX4_MSGS_VERSION=v1.17.0
WS="${ROS_WS:-$HOME/drone_ws}"
PX4_DIR="${PX4_AUTOPILOT_DIR:-$HOME/PX4-Autopilot}"

sudo apt-get update
sudo apt-get install -y curl software-properties-common
sudo add-apt-repository -y universe
curl -fsSL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
rm -f /tmp/ros2-apt-source.deb

sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git python3-colcon-common-extensions python3-pip python3-pytest \
  python3-rosdep python3-yaml python3-vcstool python3-matplotlib ros-jazzy-ros-base ros-jazzy-ros-gz \
  ros-jazzy-nav-msgs wget curl

if [[ ! -d "${PX4_DIR}/.git" ]]; then
  git clone --branch "${PX4_VERSION}" --recursive https://github.com/PX4/PX4-Autopilot.git "${PX4_DIR}"
else
  git -C "${PX4_DIR}" fetch --tags
  git -C "${PX4_DIR}" checkout "${PX4_VERSION}"
  git -C "${PX4_DIR}" submodule update --init --recursive
fi

bash "${PX4_DIR}/Tools/setup/ubuntu.sh" --no-nuttx
make -C "${PX4_DIR}" px4_sitl_default

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "${TMP_DIR}"' EXIT
git clone --depth 1 --branch "${MICRO_XRCE_AGENT_VERSION}" \
  https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "${TMP_DIR}/agent"
cmake -S "${TMP_DIR}/agent" -B "${TMP_DIR}/agent/build"
cmake --build "${TMP_DIR}/agent/build" -j"$(nproc)"
sudo cmake --install "${TMP_DIR}/agent/build"
sudo ldconfig

python3 -m pip install --user --break-system-packages pymavlink

mkdir -p "${WS}/src"
if [[ ! -d "${WS}/src/px4_msgs/.git" ]]; then
  git clone --branch "${PX4_MSGS_VERSION}" https://github.com/PX4/px4_msgs.git "${WS}/src/px4_msgs"
else
  git -C "${WS}/src/px4_msgs" fetch --tags
  git -C "${WS}/src/px4_msgs" checkout --detach "${PX4_MSGS_VERSION}"
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET="${WS}/src/drone-car-follower"
if [[ "${REPO_ROOT}" != "${TARGET}" ]]; then
  rm -rf "${TARGET}"
  cp -a "${REPO_ROOT}" "${TARGET}"
fi

source /opt/ros/${ROS_DISTRO}/setup.bash
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update
cd "${WS}"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

cat <<MSG

Setup complete.

Run:
  source /opt/ros/jazzy/setup.bash
  source ${WS}/install/setup.bash
  export PX4_AUTOPILOT_DIR=${PX4_DIR}
  ros2 launch drone_system full_stack.launch.py
MSG
