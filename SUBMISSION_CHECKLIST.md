# Submission Checklist

Deadline: Tuesday, 7 July 2026 at 5:00 pm.

## Engineering implementation

- [x] PX4 SITL drone runs in Gazebo.
- [x] The car follows a repeating closed path.
- [x] The drone arms and takes off to 20 m.
- [x] The drone follows the car at a fixed offset.
- [x] `/car/position` is published.
- [x] `/drone/waypoint` is accepted.
- [x] Follower logic is isolated between the required topics.
- [x] The follower does not read the car trajectory plugin's internal parameters.
- [x] The entire stack starts through `full_stack.launch.py`.

## Failure handling

- [x] A car-position gap greater than 200 ms commands hover and logs an error.
- [x] PX4 arming is retried three times before shutdown.
- [x] A position jump greater than 5 m is rejected.
- [x] A Gazebo real-time factor below 0.8 is reported every five seconds.
- [x] All thresholds are stored in `drone_system/config/params.yaml`.

## Logging, plots, and CI

- [x] Structured JSONL failure logging is implemented.
- [x] `tools/log_summary.py` is implemented.
- [x] `tools/plot_run.py` generates all four required plots.
- [x] The Docker image builds successfully.
- [x] All 19 repository tests pass.
- [x] The local 60-second integration test passes.
- [x] The GitHub Actions integration workflow passes.
- [x] GitHub Actions uploads the integration log and plots.
- [x] Public runtime evidence is included under `runtime_logs/`.

## Documentation

- [x] The README contains build and launch instructions.
- [x] `ANALYSIS.md` answers all four required questions.
- [x] Requirements traceability is included.
- [x] Validation evidence is documented.
- [x] The public repository is accessible.

## Final submission

- [ ] Send the submission email to `info@invictron.in`.
- [ ] Include the public repository link.
- [ ] Include one paragraph describing the hardest part.
- [ ] Include two available 30-minute live-review times with time zone.

Repository:

`https://github.com/MeetAmin1/drone-car-follower`

Required native entry point:

```bash
ros2 launch drone_system full_stack.launch.py
```