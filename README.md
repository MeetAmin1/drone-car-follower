# PX4 ROS 2 Drone-Car Follower

A complete ROS 2 system in which a PX4 SITL x500 takes off to 20 m and follows a Gazebo car on a repeating figure-eight path at a fixed 8 m horizontal trailing offset. The follower is intentionally isolated between `/car/position` and `/drone/waypoint`; it never queries Gazebo services, model-state parameters, or the car plugin's internal trajectory.

## Pinned stack

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic (`gz-sim8`)
- PX4 v1.17.0
- `px4_msgs` tag `v1.17.0`
- Micro XRCE-DDS Agent v2.4.3

PX4 and `px4_msgs` are pinned to matching versions because incompatible message definitions can prevent DDS topic communication.

## Reproducible Docker path

Requirements: Linux with Docker Engine, or Docker Desktop with host networking enabled, plus at least 4 CPU cores, 20 GB free disk, and 8 GB RAM recommended.

```bash
git clone https://github.com/MeetAmin1/drone-car-follower
cd drone-car-follower
docker build --progress=plain -t drone-car-follower .
```

Run the required launch command headlessly inside the container:

```bash
docker run --rm -it --init --network host --shm-size 1g \
  drone-car-follower \
  ros2 launch drone_system full_stack.launch.py
```

Headless mode is the launch-file default, and the container defaults to the same launch command, so this is equivalent:

```bash
docker run --rm -it --init --network host --shm-size 1g drone-car-follower
```

The first build is intentionally heavy because it installs and compiles the complete PX4, Gazebo, and ROS 2 stack. Later builds can reuse Docker cache.

### Persistent integration log and plots

A container started with `--rm` loses files stored only inside the container. Use a mounted artifact directory for a reproducible 60-second validation run:

```bash
rm -rf artifacts && mkdir -p artifacts
docker run --rm --init --network host --shm-size 1g \
  -v "$PWD/artifacts:/artifacts" \
  -e DRONE_SYSTEM_LOG=/artifacts/ci_run.jsonl \
  drone-car-follower \
  bash -lc '/workspace/src/drone-car-follower/tools/run_integration.sh 60'
```

This command prints the log summary and CI result, then saves the JSONL log and four plots under `artifacts/`.

### Optional Gazebo GUI on Linux/X11

```bash
xhost +si:localuser:root
docker run --rm -it --init --network host --shm-size 1g \
  -e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  drone-car-follower \
  ros2 launch drone_system full_stack.launch.py headless:=false
xhost -si:localuser:root
```

## Native Ubuntu 24.04 path

Run the bootstrap from the cloned repository:

```bash
bash scripts/bootstrap_host.sh
source /opt/ros/jazzy/setup.bash
source ~/drone_ws/install/setup.bash
export PX4_AUTOPILOT_DIR=~/PX4-Autopilot
```

Then start everything with the single required command:

```bash
ros2 launch drone_system full_stack.launch.py
```

No QGroundControl window is required. A minimal MAVLink GCS heartbeat is part of the stack so headless SITL can pass the normal GCS arming check.

## Expected behaviour

1. Gazebo starts the `default` world with a box car driven by a Gazebo system plugin on a closed figure-eight.
2. PX4 SITL spawns an x500 and connects to ROS 2 through Micro XRCE-DDS.
3. The controller streams offboard setpoints, waits for PX4 preflight health, requests offboard mode, arms, and climbs to 20 m.
4. After reaching 20 m, the drone follows the car at an 8 m trailing offset.
5. `/car/position` is produced from a dedicated car odometry stream; `/drone/waypoint` is produced by `follower_node` and consumed by `px4_offboard_node`.
6. Structured logs are written to `logs/run.jsonl` unless `log_file:=...` or `DRONE_SYSTEM_LOG` overrides the path.

Useful checks from another sourced terminal:

```bash
ros2 topic hz /car/position
ros2 topic echo /drone/waypoint --once
ros2 topic echo /drone/position --once
```

## Required failure handling

Every required threshold is defined in [`drone_system/config/params.yaml`](drone_system/config/params.yaml), not buried in code.

| Failure | Detection and response |
|---|---|
| `/car/position` gap greater than 0.2 s | `follower_node` logs `CAR_POSITION_TIMEOUT` with an ISO timestamp and publishes the current drone position as the hover waypoint. |
| PX4 does not arm | One initial arm command plus three configured retries; after the final retry window, `PX4_ARM_FAILED` is logged and the controller exits, causing the launch system to shut down all processes. |
| Car position jumps more than 5 m in one update | The sample is discarded, `CAR_POSITION_JUMP` is logged, and the last valid car position remains active. |
| Gazebo RTF falls below 0.8 | `gazebo_monitor_node` estimates RTF from `/clock` and wall time and logs `GAZEBO_RTF_LOW` every configured 5 s until recovery. |

Each event is one JSON line containing `timestamp`, `severity`, `component`, `event_type`, and a plain-English `description`.

## Logs and plots

Summarize a run:

```bash
python3 tools/log_summary.py artifacts/ci_run.jsonl
```

Validate the final integration window:

```bash
python3 tools/ci_check.py artifacts/ci_run.jsonl --final-window-s 30 --minimum-altitude-m 1
```

Generate all four required plots:

```bash
python3 tools/plot_run.py artifacts/ci_run.jsonl --out-dir artifacts/plots
```

Outputs:

- `xy_paths.png`
- `message_arrival_rate.png`
- `gazebo_rtf.png`
- `drone_altitude.png`

## Validation status

The corrected stack was executed in Docker on 6 July 2026. The image built successfully, all 19 repository tests passed, PX4 completed preflight checks, entered offboard mode, armed, took off, reached 20 m, and enabled follow mode. A persistent 75-second run produced 74.0 seconds of telemetry and passed the CI check: the final 30-second altitude stayed above 1 m, car updates remained active, and no final-window errors occurred.

The saved evidence is under `runtime_logs/`. One startup `CAR_POSITION_TIMEOUT` occurred when the stream gap reached 0.241 s; the node commanded hover as required and resumed follow mode when updates recovered. This is an exercised safety response, not a hidden failure.

## CI

[`.github/workflows/integration_test.yml`](.github/workflows/integration_test.yml) builds the Docker image, runs the stack for 60 seconds, validates at least 50 seconds of telemetry, checks that final-window altitude is strictly above 1 m, verifies an active car-position stream, rejects any error in the final 30 seconds, and uploads the JSONL log and plots as artifacts.

## Unit tests

Pure safety and coordinate-conversion logic is kept outside ROS imports and tested directly:

```bash
pytest -q
```

The Docker build also runs this test suite and fails if any test fails.

## Repository map

- `drone_system/launch/full_stack.launch.py` — single launch entry point
- `drone_system/drone_system/follower_node.py` — timeout, jump rejection, and offset waypoint generation
- `drone_system/drone_system/px4_offboard_node.py` — preflight, arm/takeoff/follow state machine, and ENU/NED conversion
- `drone_system/drone_system/car_position_node.py` — dedicated car odometry stream to `/car/position`
- `car_motion_plugin/` — Gazebo figure-eight motion plugin
- `drone_system/config/params.yaml` — failure thresholds and operating parameters
- `tools/` — log summary, plotting, CI validation, and integration runner
- `runtime_logs/` — validated JSONL run and four generated plots
- `ANALYSIS.md` — required written analysis and explicit limitations
- `REQUIREMENTS_TRACEABILITY.md` — requirement-by-requirement implementation map
- `VALIDATION_REPORT.md` — static and full-stack validation evidence
- `EMAIL_SUBMISSION.txt` — submission email draft

## Known limitations

The follower is feed-forward, the car dynamics are kinematic, and the validation covers one nominal Docker run plus unit-tested safety logic rather than a large seeded fault-injection campaign. See [`ANALYSIS.md`](ANALYSIS.md) for the detailed limitations and proposed next step.
