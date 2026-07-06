from __future__ import annotations

import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter

from .core import FollowerPlanner, Point3, PositionValidator, StaleDetector
from .structured_logging import StructuredLog


class FollowerNode(Node):
    """Validates car positions and publishes fixed-offset drone waypoints."""

    def __init__(self) -> None:
        super().__init__("follower_node")
        # Values are mandatory and supplied by config/params.yaml.
        self.declare_parameter("car_timeout_s", Parameter.Type.DOUBLE)
        self.declare_parameter("max_position_jump_m", Parameter.Type.DOUBLE)
        self.declare_parameter("follow_offset_m", Parameter.Type.DOUBLE)
        self.declare_parameter("target_altitude_m", Parameter.Type.DOUBLE)
        self.declare_parameter("min_heading_speed_mps", Parameter.Type.DOUBLE)
        self.declare_parameter("waypoint_publish_rate_hz", Parameter.Type.DOUBLE)
        self.declare_parameter("log_file", "")

        car_timeout_s = float(self.get_parameter("car_timeout_s").value)
        max_jump_m = float(self.get_parameter("max_position_jump_m").value)
        publish_rate_hz = float(self.get_parameter("waypoint_publish_rate_hz").value)

        self.validator = PositionValidator(max_jump_m=max_jump_m)
        self.stale_detector = StaleDetector(timeout_s=car_timeout_s)
        self.planner = FollowerPlanner(
            offset_m=float(self.get_parameter("follow_offset_m").value),
            altitude_m=float(self.get_parameter("target_altitude_m").value),
            min_heading_speed_mps=float(self.get_parameter("min_heading_speed_mps").value),
        )
        self.log = StructuredLog(
            "follower_node",
            str(self.get_parameter("log_file").value),
            self.get_logger(),
        )

        self.car_sub = self.create_subscription(PoseStamped, "/car/position", self._car_callback, 20)
        self.drone_sub = self.create_subscription(PoseStamped, "/drone/position", self._drone_callback, 20)
        self.waypoint_pub = self.create_publisher(PoseStamped, "/drone/waypoint", 20)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish_waypoint)

        self.current_car: Optional[Point3] = None
        self.previous_car: Optional[Point3] = None
        self.current_drone: Optional[Point3] = None
        self.last_car_monotonic: Optional[float] = None
        self.previous_car_monotonic: Optional[float] = None
        self.last_waypoint: Optional[Point3] = None
        self.dropout_active = False

        self.log.event("INFO", "FOLLOWER_STARTED", "Follower node started and is waiting for car position data.")

    def _car_callback(self, msg: PoseStamped) -> None:
        now = time.monotonic()
        sample = Point3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        decision = self.validator.update(sample)

        if not decision.accepted:
            if decision.reason == "jump":
                self.log.event(
                    "WARNING",
                    "CAR_POSITION_JUMP",
                    (
                        f"Discarded car position update because the one-step jump was "
                        f"{decision.jump_m:.3f} m, above the configured limit; holding the last valid position."
                    ),
                    jump_m=round(decision.jump_m, 6),
                )
            else:
                self.log.event(
                    "WARNING",
                    "CAR_POSITION_INVALID",
                    "Discarded a non-finite car position update and held the last valid position.",
                )
            return

        self.previous_car = self.current_car
        self.previous_car_monotonic = self.last_car_monotonic
        self.current_car = decision.point
        self.last_car_monotonic = now
        self.stale_detector.mark(now)

        if self.dropout_active:
            self.dropout_active = False
            self.log.event("INFO", "CAR_POSITION_RECOVERED", "Car position updates recovered; resuming follow mode.")

    def _drone_callback(self, msg: PoseStamped) -> None:
        self.current_drone = Point3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def _publish_waypoint(self) -> None:
        now = time.monotonic()
        # A timeout is a stopped stream, not merely a stream that has not started yet.
        if self.current_car is None:
            self._publish_hover()
            return

        if self.stale_detector.is_stale(now):
            if not self.dropout_active:
                self.dropout_active = True
                age_s = None if self.last_car_monotonic is None else now - self.last_car_monotonic
                age_text = "no car position has been received" if age_s is None else f"the last update was {age_s:.3f} s ago"
                self.log.event(
                    "ERROR",
                    "CAR_POSITION_TIMEOUT",
                    f"Car position stream timed out ({age_text}); commanding the drone to hover at its current position.",
                    age_s=None if age_s is None else round(age_s, 6),
                )
            self._publish_hover()
            return

        dt_s: Optional[float] = None
        if self.previous_car_monotonic is not None and self.last_car_monotonic is not None:
            dt_s = self.last_car_monotonic - self.previous_car_monotonic

        waypoint = self.planner.waypoint(self.current_car, self.previous_car, dt_s)
        self.last_waypoint = waypoint
        self._publish_point(waypoint)

    def _publish_hover(self) -> None:
        if self.current_drone is not None:
            hover = Point3(self.current_drone.x, self.current_drone.y, self.current_drone.z)
            self.last_waypoint = hover
            self._publish_point(hover)
        elif self.last_waypoint is not None:
            self._publish_point(self.last_waypoint)

    def _publish_point(self, point: Point3) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = point.x
        msg.pose.position.y = point.y
        msg.pose.position.z = point.z
        msg.pose.orientation.w = 1.0
        self.waypoint_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
