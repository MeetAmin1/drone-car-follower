from __future__ import annotations

from collections import deque
import time
from typing import Deque, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64

from .core import Point3
from .structured_logging import StructuredLog


class TelemetryLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__("telemetry_logger_node")
        self.declare_parameter("telemetry_rate_hz", Parameter.Type.DOUBLE)
        self.declare_parameter("arrival_rate_window_s", Parameter.Type.DOUBLE)
        self.declare_parameter("log_file", "")

        telemetry_rate_hz = float(self.get_parameter("telemetry_rate_hz").value)
        self.arrival_window_s = float(self.get_parameter("arrival_rate_window_s").value)
        self.log = StructuredLog(
            "telemetry_logger_node",
            str(self.get_parameter("log_file").value),
            self.get_logger(),
        )

        self.car: Optional[Point3] = None
        self.drone: Optional[Point3] = None
        self.rtf: Optional[float] = None
        self.arrivals: Deque[float] = deque()

        self.create_subscription(PoseStamped, "/car/position", self._car_callback, 50)
        self.create_subscription(PoseStamped, "/drone/position", self._drone_callback, 50)
        self.create_subscription(Float64, "/system/gazebo_rtf", self._rtf_callback, 20)
        self.timer = self.create_timer(1.0 / telemetry_rate_hz, self._record)
        self.log.event("INFO", "TELEMETRY_STARTED", "Telemetry recording started.")

    def _car_callback(self, msg: PoseStamped) -> None:
        now = time.monotonic()
        self.arrivals.append(now)
        self.car = Point3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def _drone_callback(self, msg: PoseStamped) -> None:
        self.drone = Point3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def _rtf_callback(self, msg: Float64) -> None:
        self.rtf = float(msg.data)

    def _record(self) -> None:
        now = time.monotonic()
        cutoff = now - self.arrival_window_s
        while self.arrivals and self.arrivals[0] < cutoff:
            self.arrivals.popleft()
        arrival_rate_hz = len(self.arrivals) / self.arrival_window_s

        self.log.telemetry(
            car_x=None if self.car is None else self.car.x,
            car_y=None if self.car is None else self.car.y,
            car_z=None if self.car is None else self.car.z,
            drone_x=None if self.drone is None else self.drone.x,
            drone_y=None if self.drone is None else self.drone.y,
            drone_z=None if self.drone is None else self.drone.z,
            car_position_rate_hz=arrival_rate_hz,
            gazebo_rtf=self.rtf,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TelemetryLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
