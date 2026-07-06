# Requirements Traceability

This file maps every assessment requirement to the implementation and its verification path. It is not a substitute for the README; it is a reviewer index.

| Assessment requirement | Implementation | Verification |
|---|---|---|
| PX4 SITL drone in Gazebo | PX4 v1.17.0 `gz_x500` is started by `drone_system/launch/full_stack.launch.py`; the custom world is `drone_system/worlds/car_follow.sdf`. | Validated Docker run in `runtime_logs/run.jsonl`, integration workflow, and `tools/run_integration.sh`. |
| Car repeats a closed path | `car_motion_plugin/src/figure_eight_motion.cpp` commands a continuous figure-eight pose. | SDF/package contract tests and visual/runtime integration check. |
| Arm, take off to 20 m, then follow at fixed offset | `px4_offboard_node.py` owns warmup, offboard, arming, takeoff, and follow states; `follower_node.py` produces an 8 m horizontal behind-car waypoint at 20 m altitude. | `tests/test_core.py`, `runtime_logs/plots/drone_altitude.png`, and the passing CI altitude check. |
| Publish car position on `/car/position` | Gazebo's model-attached odometry publisher emits `/car/odometry`; `car_position_node.py` relays it to `/car/position`. | `tests/test_repository_contract.py` and the passing final-window car-message-rate check. |
| Accept waypoint commands on `/drone/waypoint` | `px4_offboard_node.py` subscribes to `/drone/waypoint` and converts ROS ENU setpoints to PX4 NED. | Source contract test, ENU/NED unit test, and integration run. |
| Follower logic sits between the two topics | `follower_node.py` subscribes only to `/car/position` and `/drone/position`, then publishes `/drone/waypoint`. | Topic-isolation test in `tests/test_repository_contract.py`. |
| Do not read direct Gazebo/car position parameters | The follower has no Gazebo import, service, model-state call, or knowledge of the scripted trajectory. Car location arrives as a dedicated odometry sensor stream. | Static isolation tests and architecture review. |
| One launch command | `ros2 launch drone_system full_stack.launch.py` starts Gazebo, PX4, DDS agent, bridge, heartbeat, relay, follower, controller, monitor, and telemetry logger. | README, launch contract test, Docker entrypoint. |
| Car gap greater than 200 ms -> hover + timestamped error | `StaleDetector` and `follower_node.py`; event type `CAR_POSITION_TIMEOUT`. | `tests/test_core.py`; threshold is `car_timeout_s: 0.2`. |
| PX4 arm failure -> retry three times, then clean shutdown | `ArmRetryPolicy` and the controller state machine send one initial command plus three retries; event type `PX4_ARM_FAILED`; nonzero node exit triggers launch shutdown. | `tests/test_core.py`; `arm_retry_count: 3`. |
| Position jump greater than 5 m -> discard, warn, hold last valid | `PositionValidator` and event type `CAR_POSITION_JUMP`. | `tests/test_core.py`; `max_position_jump_m: 5.0`. |
| RTF below 0.8 -> warning every 5 s until recovery | `RtfEstimator` and `gazebo_monitor_node.py`; events `GAZEBO_RTF_LOW` and `GAZEBO_RTF_RECOVERED`. | `tests/test_core.py`; `rtf_min: 0.8`, `rtf_warning_interval_s: 5.0`. |
| Thresholds in `config/params.yaml` | Runtime thresholds and operating values are declared in `drone_system/config/params.yaml`; nodes intentionally provide no fallback for required values. | Parameter contract test. |
| Every failure log has ISO timestamp, severity, component, description | `structured_logging.py` writes atomic JSONL event records with all required fields. | Log-summary tests and schema test. |
| `tools/log_summary.py` output | Prints warning count, error count, unique error types, first error time, and last error time. | `tests/test_tools.py`. |
| Docker CI runs 60 s, checks altitude, active car stream, and final-window errors, then saves logs | `.github/workflows/integration_test.yml`, `tools/run_integration.sh`, and `tools/ci_check.py`. | Workflow source contract and synthetic CI tests. |
| Four required plots | `tools/plot_run.py` writes `xy_paths.png`, `message_arrival_rate.png`, `gazebo_rtf.png`, and `drone_altitude.png`. | Synthetic tool validation and artifact checks. |
| `ANALYSIS.md`, all four questions, max two paragraphs each | `ANALYSIS.md`. | Manual review; completion status records the successful Docker integration run. |
| Submission email | `EMAIL_SUBMISSION.txt` contains the recipient, repository placeholder, hardest-part paragraph, and two availability placeholders. | `SUBMISSION_CHECKLIST.md`. |
