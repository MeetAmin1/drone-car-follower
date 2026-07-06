# Validation Report

Validation date: 6 July 2026.

## Build and static validation

- The Docker image built successfully from the repository Dockerfile.
- All 19 repository tests passed.
- Python source files and ROS 2 launch files compiled successfully.
- Package XML, Gazebo SDF, YAML configuration, workflow YAML, and shell syntax checks passed.
- PX4 v1.17.0 and `px4_msgs` v1.17.0 are pinned to compatible versions.
- The built image contains PX4 SITL, Gazebo Harmonic, Micro XRCE-DDS Agent, and the ROS 2 packages required by the system.

## Full-stack runtime validation

The complete PX4 SITL, Gazebo Harmonic, and ROS 2 stack ran successfully through the required launch entry point.

The runtime confirmed:

- Gazebo started the custom world.
- The car began its repeating figure-eight path.
- The PX4 x500 vehicle spawned successfully.
- Micro XRCE-DDS connected PX4 and ROS 2.
- `/car/position` became active.
- PX4 vehicle status was received.
- PX4 preflight checks passed.
- Offboard mode was requested.
- PX4 armed successfully.
- Takeoff was detected.
- The drone reached 20 m.
- Follow mode was enabled.

## Integration-test result

The automated 60-second integration test passed.

The final validation window confirmed:

- Drone altitude remained above 1 m.
- The car-position stream remained active.
- No error occurred during the final 30 seconds.
- The JSONL log and all four required plots were generated.

The log contains one startup `CAR_POSITION_TIMEOUT`. The update gap briefly exceeded 0.2 s, the follower commanded hover, and follow mode resumed when the stream recovered. This is the required failure response.

The timed integration runner intentionally sends SIGINT when the test duration ends. Signal-based shutdown messages therefore represent controlled test termination rather than a mission failure.

## GitHub Actions

The repository workflow builds the Docker image, runs the 60-second full-stack integration test, validates the final runtime window, and uploads the generated log and plots.

## Public evidence

- `runtime_logs/run.jsonl`
- `runtime_logs/plots/xy_paths.png`
- `runtime_logs/plots/message_arrival_rate.png`
- `runtime_logs/plots/gazebo_rtf.png`
- `runtime_logs/plots/drone_altitude.png`