from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from .core import ArmRetryPolicy, Point3, enu_to_ned, ned_to_enu
from .structured_logging import StructuredLog


PX4_OUTPUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class Px4OffboardNode(Node):
    def __init__(self) -> None:
        super().__init__("px4_offboard_node")
        # Values are mandatory and supplied by config/params.yaml.
        self.declare_parameter("control_rate_hz", Parameter.Type.DOUBLE)
        self.declare_parameter("offboard_warmup_s", Parameter.Type.DOUBLE)
        self.declare_parameter("arm_retry_count", Parameter.Type.INTEGER)
        self.declare_parameter("arm_retry_interval_s", Parameter.Type.DOUBLE)
        self.declare_parameter("offboard_retry_count", Parameter.Type.INTEGER)
        self.declare_parameter("offboard_retry_interval_s", Parameter.Type.DOUBLE)
        self.declare_parameter("takeoff_altitude_m", Parameter.Type.DOUBLE)
        self.declare_parameter("takeoff_tolerance_m", Parameter.Type.DOUBLE)
        self.declare_parameter("takeoff_stable_s", Parameter.Type.DOUBLE)
        self.declare_parameter("takeoff_timeout_s", Parameter.Type.DOUBLE)
        self.declare_parameter("waypoint_timeout_s", Parameter.Type.DOUBLE)
        self.declare_parameter("px4_status_initial_timeout_s", Parameter.Type.DOUBLE)
        self.declare_parameter("px4_preflight_timeout_s", Parameter.Type.DOUBLE)
        self.declare_parameter("px4_status_loss_timeout_s", Parameter.Type.DOUBLE)
        self.declare_parameter("log_file", "")

        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.offboard_warmup_s = float(self.get_parameter("offboard_warmup_s").value)
        self.takeoff_altitude_m = float(self.get_parameter("takeoff_altitude_m").value)
        self.takeoff_tolerance_m = float(self.get_parameter("takeoff_tolerance_m").value)
        self.takeoff_stable_s = float(self.get_parameter("takeoff_stable_s").value)
        self.takeoff_timeout_s = float(self.get_parameter("takeoff_timeout_s").value)
        self.waypoint_timeout_s = float(self.get_parameter("waypoint_timeout_s").value)
        self.px4_status_initial_timeout_s = float(
            self.get_parameter("px4_status_initial_timeout_s").value
        )
        self.px4_preflight_timeout_s = float(
            self.get_parameter("px4_preflight_timeout_s").value
        )
        self.px4_status_loss_timeout_s = float(
            self.get_parameter("px4_status_loss_timeout_s").value
        )
        self.arm_policy = ArmRetryPolicy(
            retry_count=int(self.get_parameter("arm_retry_count").value),
            retry_interval_s=float(self.get_parameter("arm_retry_interval_s").value),
        )
        self.offboard_policy = ArmRetryPolicy(
            retry_count=int(self.get_parameter("offboard_retry_count").value),
            retry_interval_s=float(self.get_parameter("offboard_retry_interval_s").value),
        )
        self.log = StructuredLog(
            "px4_offboard_node",
            str(self.get_parameter("log_file").value),
            self.get_logger(),
        )

        self.offboard_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.position_pub = self.create_publisher(PoseStamped, "/drone/position", 20)

        # PX4 1.17 marks these messages MESSAGE_VERSION=1, so the DDS
        # bridge exposes versioned topic names while retaining the same ROS type.
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self._status_callback, PX4_OUTPUT_QOS)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self._local_position_callback, PX4_OUTPUT_QOS)
        self.create_subscription(VehicleCommandAck, "/fmu/out/vehicle_command_ack", self._ack_callback, PX4_OUTPUT_QOS)
        self.create_subscription(PoseStamped, "/drone/waypoint", self._waypoint_callback, 20)

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self._tick)
        self.started_monotonic = time.monotonic()
        self.last_status_monotonic: Optional[float] = None
        self.first_status_monotonic: Optional[float] = None
        self.preflight_ready_monotonic: Optional[float] = None
        self.last_waypoint_monotonic: Optional[float] = None
        self.current_enu: Optional[Point3] = None
        self.requested_waypoint_enu: Optional[Point3] = None
        self.vehicle_status: Optional[VehicleStatus] = None
        self.state = "WARMUP"
        self.takeoff_reached_since: Optional[float] = None
        self.takeoff_started_monotonic: Optional[float] = None
        self.exit_code = 0
        self.last_arm_ack_result: Optional[int] = None

        self.log.event(
            "INFO",
            "PX4_CONTROLLER_STARTED",
            "PX4 offboard controller started; streaming takeoff setpoints before requesting offboard mode and arming.",
        )

    def _timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _status_callback(self, msg: VehicleStatus) -> None:
        now = time.monotonic()
        self.vehicle_status = msg
        self.last_status_monotonic = now
        if self.first_status_monotonic is None:
            self.first_status_monotonic = now
            self.log.event(
                "INFO",
                "PX4_STATUS_CONNECTED",
                "PX4 vehicle status stream connected; waiting for preflight health checks to pass.",
            )

    def _local_position_callback(self, msg: VehicleLocalPosition) -> None:
        if not (bool(msg.xy_valid) and bool(msg.z_valid)):
            return
        enu = ned_to_enu(Point3(float(msg.x), float(msg.y), float(msg.z)))
        self.current_enu = enu
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "world"
        out.pose.position.x = enu.x
        out.pose.position.y = enu.y
        out.pose.position.z = enu.z
        out.pose.orientation.w = 1.0
        self.position_pub.publish(out)

    def _ack_callback(self, msg: VehicleCommandAck) -> None:
        if int(msg.command) == int(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM):
            self.last_arm_ack_result = int(msg.result)

    def _waypoint_callback(self, msg: PoseStamped) -> None:
        point = Point3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        if not all(math.isfinite(v) for v in (point.x, point.y, point.z)):
            self.log.event(
                "WARNING",
                "WAYPOINT_INVALID",
                "Discarded a non-finite waypoint command and retained the previous setpoint.",
            )
            return
        self.requested_waypoint_enu = point
        self.last_waypoint_monotonic = time.monotonic()

    def _armed(self) -> bool:
        return (
            self.vehicle_status is not None
            and int(self.vehicle_status.arming_state)
            == int(VehicleStatus.ARMING_STATE_ARMED)
        )

    def _offboard_active(self) -> bool:
        return (
            self.vehicle_status is not None
            and int(self.vehicle_status.nav_state)
            == int(VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        )

    def _enter_takeoff(self, now: float) -> None:
        self.state = "TAKEOFF"
        self.takeoff_started_monotonic = now
        self.log.event(
            "INFO",
            "PX4_ARMED",
            f"PX4 armed in offboard mode; climbing to {self.takeoff_altitude_m:.1f} m before following the car.",
        )

    def _tick(self) -> None:
        now = time.monotonic()
        if self.last_status_monotonic is None:
            if (now - self.started_monotonic) > self.px4_status_initial_timeout_s:
                self._shutdown_with_error(
                    "PX4_STATUS_NOT_RECEIVED",
                    "No PX4 vehicle status arrived before the configured startup timeout; shutting down cleanly.",
                )
                return
        elif (now - self.last_status_monotonic) > self.px4_status_loss_timeout_s:
            self._shutdown_with_error(
                "PX4_STATUS_LOST",
                "PX4 vehicle status stopped arriving; shutting down cleanly because flight state is unknown.",
                age_s=round(now - self.last_status_monotonic, 6),
            )
            return

        if self.state in {"TAKEOFF", "FOLLOW"}:
            if not self._armed():
                self._shutdown_with_error(
                    "PX4_DISARMED_UNEXPECTEDLY",
                    "PX4 became disarmed during autonomous flight; shutting down the control stack cleanly.",
                )
                return
            if not self._offboard_active():
                self._shutdown_with_error(
                    "PX4_OFFBOARD_LOST",
                    "PX4 left offboard mode during autonomous flight; shutting down the control stack cleanly.",
                )
                return

        self._publish_offboard_control_mode()

        target_enu = self._select_target(now)
        self._publish_trajectory_setpoint(target_enu)

        if self.state == "WARMUP":
            preflight_ready = (
                self.vehicle_status is not None
                and bool(self.vehicle_status.pre_flight_checks_pass)
            )
            if preflight_ready:
                if self.preflight_ready_monotonic is None:
                    self.preflight_ready_monotonic = now
                    self.log.event(
                        "INFO",
                        "PX4_PREFLIGHT_READY",
                        "PX4 preflight checks passed; continuing the offboard setpoint warmup.",
                    )
                if (now - self.preflight_ready_monotonic) >= self.offboard_warmup_s:
                    self._send_offboard_mode()
                    self.state = "ARMING"
            else:
                self.preflight_ready_monotonic = None
                if (
                    self.first_status_monotonic is not None
                    and (now - self.first_status_monotonic) > self.px4_preflight_timeout_s
                ):
                    self._shutdown_with_error(
                        "PX4_PREFLIGHT_TIMEOUT",
                        "PX4 status was received, but preflight checks did not pass before the configured timeout.",
                    )
                    return

        if self.state == "ARMING":
            if self._armed() and self._offboard_active():
                self._enter_takeoff(now)
            elif self._armed():
                self.state = "OFFBOARD_CONFIRM"
                self.log.event(
                    "WARNING",
                    "PX4_ARMED_WITHOUT_OFFBOARD",
                    "PX4 armed but offboard mode was not confirmed; retrying the mode request before takeoff.",
                )
            else:
                if self.arm_policy.should_send(now):
                    command_number = self.arm_policy.record_command(now)
                    retry_number = max(0, command_number - 1)
                    # Reassert offboard mode before every bounded arming attempt.
                    self._send_offboard_mode(log_event=False)
                    self._send_arm_command()
                    if retry_number == 0:
                        description = "Sent the initial PX4 arm command."
                        event_type = "PX4_ARM_INITIAL"
                    else:
                        description = f"PX4 did not arm; sent retry {retry_number} of {self.arm_policy.retry_count}."
                        event_type = "PX4_ARM_RETRY"
                    self.log.event(
                        "WARNING" if retry_number > 0 else "INFO",
                        event_type,
                        description,
                        command_number=command_number,
                        retry_number=retry_number,
                        last_ack_result=self.last_arm_ack_result,
                    )
                elif self.arm_policy.exhausted_after_wait(now):
                    self._shutdown_with_error(
                        "PX4_ARM_FAILED",
                        (
                            f"PX4 failed to arm after the initial command and {self.arm_policy.retry_count} retries; "
                            "shutting down the stack cleanly."
                        ),
                        commands_sent=self.arm_policy.commands_sent,
                        last_ack_result=self.last_arm_ack_result,
                    )
                    return

        if self.state == "OFFBOARD_CONFIRM":
            if not self._armed():
                self._shutdown_with_error(
                    "PX4_DISARMED_BEFORE_TAKEOFF",
                    "PX4 disarmed while offboard mode was being confirmed; shutting down cleanly.",
                )
                return
            if self._offboard_active():
                self._enter_takeoff(now)
            elif self.offboard_policy.should_send(now):
                command_number = self.offboard_policy.record_command(now)
                self._send_offboard_mode(log_event=False)
                self.log.event(
                    "WARNING",
                    "PX4_OFFBOARD_RETRY",
                    "PX4 offboard mode was not confirmed; reissued the bounded mode request.",
                    command_number=command_number,
                )
            elif self.offboard_policy.exhausted_after_wait(now):
                self._shutdown_with_error(
                    "PX4_OFFBOARD_FAILED",
                    "PX4 did not enter offboard mode after the configured retries; shutting down cleanly.",
                    commands_sent=self.offboard_policy.commands_sent,
                )
                return

        if self.state == "TAKEOFF":
            if (
                self.takeoff_started_monotonic is not None
                and (now - self.takeoff_started_monotonic) > self.takeoff_timeout_s
            ):
                self._shutdown_with_error(
                    "TAKEOFF_TIMEOUT",
                    f"Drone did not reach {self.takeoff_altitude_m:.1f} m before the configured takeoff timeout.",
                    current_altitude_m=None if self.current_enu is None else round(self.current_enu.z, 6),
                )
                return

        if self.state == "TAKEOFF" and self.current_enu is not None:
            if self.current_enu.z >= (self.takeoff_altitude_m - self.takeoff_tolerance_m):
                if self.takeoff_reached_since is None:
                    self.takeoff_reached_since = now
                elif (now - self.takeoff_reached_since) >= self.takeoff_stable_s:
                    self.state = "FOLLOW"
                    self.log.event(
                        "INFO",
                        "TAKEOFF_COMPLETE",
                        f"Drone reached the configured takeoff altitude of {self.takeoff_altitude_m:.1f} m; follow mode enabled.",
                    )
            else:
                self.takeoff_reached_since = None

    def _shutdown_with_error(self, event_type: str, description: str, **fields) -> None:
        if self.exit_code != 0:
            return
        self.log.event("ERROR", event_type, description, **fields)
        self.exit_code = 2
        rclpy.shutdown()

    def _select_target(self, now: float) -> Point3:
        takeoff_x = 0.0 if self.current_enu is None else self.current_enu.x
        takeoff_y = 0.0 if self.current_enu is None else self.current_enu.y
        takeoff = Point3(takeoff_x, takeoff_y, self.takeoff_altitude_m)
        if self.state != "FOLLOW":
            return takeoff

        if self.requested_waypoint_enu is not None and self.last_waypoint_monotonic is not None:
            if (now - self.last_waypoint_monotonic) <= self.waypoint_timeout_s:
                return self.requested_waypoint_enu

        if self.current_enu is not None:
            return self.current_enu
        return takeoff

    def _publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()
        msg.timestamp = self._timestamp_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.offboard_pub.publish(msg)

    def _publish_trajectory_setpoint(self, target_enu: Point3) -> None:
        target_ned = enu_to_ned(target_enu)
        msg = TrajectorySetpoint()
        msg.timestamp = self._timestamp_us()
        msg.position = [target_ned.x, target_ned.y, target_ned.z]
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = math.nan
        msg.yawspeed = math.nan
        self.setpoint_pub.publish(msg)

    def _send_offboard_mode(self, log_event: bool = True) -> None:
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )
        if log_event:
            self.log.event(
                "INFO",
                "OFFBOARD_REQUESTED",
                "Requested PX4 offboard mode after the configured warmup period.",
            )

    def _send_arm_command(self) -> None:
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )

    def _publish_vehicle_command(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
    ) -> None:
        msg = VehicleCommand()
        msg.timestamp = self._timestamp_us()
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4OffboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)
