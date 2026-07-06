# Analysis

## Completion status

The repository implements the requested PX4 SITL, Gazebo car, ROS 2 follower, failure handling, structured logs, plotting tools, Docker build, and CI workflow. The corrected stack was executed in Docker on 6 July 2026: all 19 tests passed, PX4 completed preflight checks, entered offboard mode, armed, took off, reached 20 m, and enabled follow mode. A persistent 75-second run produced 74.0 seconds of telemetry, four plots, and a passing final-window CI result. One startup car-stream gap of 0.241 s exercised the required hover response and recovered when updates resumed.

## Q1 - The Conflict

I would ask the AI team whether 150 ms is fixed transport latency or variable end-to-end age, whether each detection carries a capture timestamp, the dropout/outlier distribution, whether velocity or track identity is available, and whether the +/-0.8 m figure is one-sigma, a bound, or an average. Fixed timestamped latency supports a predictor that propagates the car state to the control time; variable latency requires age-aware rejection and a larger uncertainty model; frequent outliers require gating and track management rather than a simple low-pass filter. I would also ask whether the camera frame is calibrated to the control frame and how often that calibration changes, because an uncertain transform belongs in the estimator rather than being treated as controller noise.

I would ask the control team what plant, speed range, gains, and evidence produced the 50 ms requirement; whether 50 ms means a new measurement or merely a 20 Hz control setpoint; what latency and jitter margins are tolerable; and whether the controller can consume position, velocity, covariance, and timestamps. If it only needs a 20 Hz setpoint, I would run the estimator asynchronously at camera rate and publish predicted states at 20 Hz. If it truly requires independent measurements every 50 ms, the camera cannot satisfy it and the architecture needs another sensor or a controller redesign; fabricating repeated camera samples would hide stale data and preserve the oscillation risk.

## Q2 - The Bug

I would fix it immediately in a small reviewed change, add a frame-contract test, and tell the affected owners before merging. Fixing it silently is reckless because downstream logs, tuning, and recorded data may already be contaminated; reporting it and waiting is also weak when the defect is understood, low-risk, and blocks correct integration. The notification should state the wrong and correct frames, affected versions or data, the transform applied, and the test that prevents recurrence.

Two hours before a demo, the principle does not change but the deployment path does. I would freeze the main branch, reproduce the defect, make the smallest reversible patch, run the frame and smoke tests, and deploy only with an explicit go/no-go decision from the demo owner; otherwise I would use the last known-good build and disclose the limitation. A rushed unreviewed "10-minute fix" that breaks the demo is not heroism, it is uncontrolled change.

## Q3 - Your Weaknesses

First, the follower is feed-forward and uses a finite-difference heading estimate; it has no explicit acceleration model or closed-loop offset-error controller. The scripted car peaks at roughly 3.5 m/s, so a 200 ms stale-input boundary already corresponds to about 0.7 m of unobserved car travel before the hover response. Second, the car motion is kinematic and open-loop: there is no tyre model, obstacle interaction, wheel slip, or perception pipeline, so simulation success says little about real car dynamics. Third, the integration is pinned to Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, PX4 v1.17.0, px4_msgs v1.17.0, and Micro XRCE-DDS Agent v2.4.3; a version mismatch can stop topic compatibility or plugin loading before control logic runs.

The safety behaviour is deliberately narrow. Hovering on stale car data depends on valid PX4 local position, the structured JSONL logger uses concurrent append rather than a logging daemon, and CI only proves a 60-second nominal run plus final-window checks. It does not yet perform hundreds of seeded fault-injection runs, test process restarts, quantify offset error distributions, or validate recovery after Gazebo/PX4 reconnection.

## Q4 - One More Week

I would build a deterministic fault-injection and soak-test harness that repeatedly delays, drops, corrupts, and frame-shifts `/car/position`, denies arming, throttles Gazebo, and kills/restarts processes while asserting state-machine and flight invariants. That single change attacks the largest reliability gap: today the handlers exist and pure logic is unit-tested, but the distributed timing and recovery paths are not statistically exercised. A week spent adding more controller sophistication would be less valuable than proving the existing safety claims over hundreds of reproducible runs with saved seeds and failure artifacts.
