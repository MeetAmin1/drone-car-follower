#!/usr/bin/env bash
set -eo pipefail

# Generated ROS, colcon and PX4 setup scripts are not nounset-safe.
set +u
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
set -u

export PX4_AUTOPILOT_DIR="${PX4_AUTOPILOT_DIR:-/opt/PX4-Autopilot}"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

PX4_GZ_ENV="${PX4_AUTOPILOT_DIR}/build/px4_sitl_default/rootfs/gz_env.sh"

if [[ ! -f "${PX4_GZ_ENV}" ]]; then
  echo "ERROR: missing PX4 Gazebo environment file: ${PX4_GZ_ENV}" >&2
  exit 2
fi

set +u
source "${PX4_GZ_ENV}"
set -u

exec "$@"