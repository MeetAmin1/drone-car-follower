# Validation Report

Validation date: 6 July 2026.

## Static and build validation

- Docker image built successfully from the repository Dockerfile.
- All **19 tests passed** during the Docker build and again against the extracted repository.
- Python source and launch files compiled successfully.
- XML package manifests and the Gazebo SDF world parsed successfully.
- YAML configuration and workflow files parsed successfully.
- Shell syntax checks passed for the native bootstrap, Docker entrypoint, and integration runner.
- The container smoke test confirmed the PX4 executable, generated `gz_env.sh`, PX4 Gazebo `server.config`, and ROS package installation were present.

## Full-stack Docker validation

The corrected PX4 SITL + Gazebo Harmonic + ROS 2 stack was executed successfully in Docker Desktop using the required launch file. The runtime log confirmed:

- Gazebo world ready and x500 spawned.
- Micro XRCE-DDS agent connected and PX4 ROS 2 topics were created.
- PX4 vehicle status connected.
- PX4 preflight checks passed.
- Offboard mode requested and accepted.
- PX4 armed by external command.
- Takeoff detected.
- The drone reached the configured 20.0 m altitude.
- Follow mode was enabled.
- PX4, Gazebo, bridge, and ROS nodes shut down cleanly after interruption.

## Persistent integration result

A 75-second automated run produced **74.0 seconds of telemetry** in `runtime_logs/run.jsonl` and generated all four required plots:

- `runtime_logs/plots/xy_paths.png`
- `runtime_logs/plots/message_arrival_rate.png`
- `runtime_logs/plots/gazebo_rtf.png`
- `runtime_logs/plots/drone_altitude.png`

The CI validator passed. During the final 30 seconds:

- Drone altitude stayed between approximately **19.968 m and 20.050 m**.
- Car-position arrival rate remained **50 Hz**.
- Gazebo real-time factor stayed between approximately **0.9986 and 0.9993**.
- No error events occurred.

The log summary reported zero warnings and one error event: `CAR_POSITION_TIMEOUT` during startup when the update gap reached 0.241 s. The follower commanded hover and logged `CAR_POSITION_RECOVERED` when updates resumed about 0.08 s later. This confirms the required >200 ms failure response operated correctly and did not prevent the mission.

## Remaining release checks

The local implementation and integration run are validated. Before submission, the public GitHub repository still needs to be created, personal placeholders must be replaced, and the GitHub Actions `integration-test` workflow must be observed passing from the clean public repository.
