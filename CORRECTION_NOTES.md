# PX4 1.17 Runtime Correction

## Root causes

The original launch started Gazebo directly but did not set PX4's `GZ_SIM_SERVER_CONFIG_PATH`. Gazebo therefore omitted PX4-required world systems for IMU, air pressure, magnetometer, NavSat, contacts, and rendering. The x500 model spawned, but PX4 reported missing sensors and EKF data, so `pre_flight_checks_pass` never became true.

The original controller also used pre-PX4-1.17 ROS 2 output names. In PX4 1.17, `VehicleStatus` and `VehicleLocalPosition` are version 1 messages exposed as `/fmu/out/vehicle_status_v1` and `/fmu/out/vehicle_local_position_v1`.

PX4 was initially launched through `make`. On shutdown, the wrapper could exit while leaving the child PX4 process alive, producing `PX4 server already running for instance 0` on the next run.

The Docker entrypoint initially enabled Bash nounset mode before sourcing generated ROS and PX4 setup scripts. That caused startup to fail on an unset `AMENT_TRACE_SETUP_FILES` variable.

## Corrections

- Load `/opt/PX4-Autopilot/src/modules/simulation/gz_bridge/server.config` before Gazebo starts.
- Add PX4's generated Gazebo plugin directory to `GZ_SIM_SYSTEM_PLUGIN_PATH`.
- Use the PX4 1.17 versioned ROS 2 output topic names.
- Wait for `VehicleStatus.pre_flight_checks_pass` before starting the bounded arm sequence.
- Execute the PX4 SITL binary directly with `PX4_SYS_AUTOSTART=4001` and `PX4_SIM_MODEL=gz_x500`, avoiding orphaned child processes.
- Pin `px4_msgs` to the exact `v1.17.0` tag to match PX4 Autopilot `v1.17.0`.
- Declare ROS parameters with explicit types, removing Jazzy parameter declaration warnings.
- Temporarily disable nounset while sourcing generated ROS, colcon, and PX4 environment scripts, then re-enable it for the entrypoint itself.

## Validation after correction

- Docker image built successfully.
- Repository tests: **19 passed**.
- Entrypoint and PX4/Gazebo file smoke checks passed.
- PX4 preflight checks passed, the vehicle armed, took off, reached 20 m, and entered follow mode.
- A persistent 75-second run generated 74.0 seconds of telemetry and all four required plots.
- The final-window CI check passed.
