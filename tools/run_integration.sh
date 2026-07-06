#!/usr/bin/env bash
set -Eeuo pipefail

DURATION_S="${1:-60}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_FILE="${DRONE_SYSTEM_LOG:-${REPO_ROOT}/logs/ci_run.jsonl}"
mkdir -p "$(dirname "${LOG_FILE}")"
: > "${LOG_FILE}"

source /opt/ros/jazzy/setup.bash
source "${ROS_WS:-/workspace}/install/setup.bash"

export DRONE_SYSTEM_LOG="${LOG_FILE}"
set +e
timeout --signal=INT --kill-after=15s "${DURATION_S}s" \
  ros2 launch drone_system full_stack.launch.py headless:=true log_file:="${LOG_FILE}"
LAUNCH_STATUS=$?
set -e

# timeout returns 124 after intentionally stopping the 60-second run.
if [[ "${LAUNCH_STATUS}" -ne 0 && "${LAUNCH_STATUS}" -ne 124 && "${LAUNCH_STATUS}" -ne 130 ]]; then
  echo "Launch exited unexpectedly with status ${LAUNCH_STATUS}."
  exit "${LAUNCH_STATUS}"
fi

python3 "${REPO_ROOT}/tools/log_summary.py" "${LOG_FILE}"
python3 "${REPO_ROOT}/tools/ci_check.py" "${LOG_FILE}" --final-window-s 30 --minimum-altitude-m 1
python3 "${REPO_ROOT}/tools/plot_run.py" "${LOG_FILE}" --out-dir "$(dirname "${LOG_FILE}")/plots"
