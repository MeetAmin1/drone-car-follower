# Architecture

```mermaid
flowchart LR
  GZ[Gazebo Harmonic\ncar + x500] -->|car odometry| B[ros_gz_bridge]
  B -->|nav_msgs/Odometry| C[car_position_node]
  C -->|/car/position| F[follower_node]
  P[PX4 local position] -->|/drone/position| F
  F -->|/drone/waypoint| O[px4_offboard_node]
  O -->|OffboardControlMode\nTrajectorySetpoint\nVehicleCommand| PX4[PX4 SITL]
  PX4 -->|VehicleStatus\nVehicleLocalPosition| O
  GZ -->|/clock| R[gazebo_monitor_node]
  C --> T[telemetry_logger_node]
  O --> T
  R --> T
  T --> L[(JSONL log)]
```

The car uses Gazebo's dedicated odometry-publisher system; the follower has no direct Gazebo API or model-state access. It consumes only `/car/position`, validates that stream, and publishes `/drone/waypoint`. The PX4 node is the sole owner of flight-state transitions and ENU/NED conversion.
