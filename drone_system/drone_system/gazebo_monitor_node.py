from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64

from .core import RtfEstimator
from .structured_logging import StructuredLog


class GazeboMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_monitor_node")
        # Values are mandatory and supplied by config/params.yaml.
        self.declare_parameter("rtf_min", Parameter.Type.DOUBLE)
        self.declare_parameter("rtf_warning_interval_s", Parameter.Type.DOUBLE)
        self.declare_parameter("rtf_window_s", Parameter.Type.DOUBLE)
        self.declare_parameter("monitor_rate_hz", Parameter.Type.DOUBLE)
        self.declare_parameter("log_file", "")

        self.rtf_min = float(self.get_parameter("rtf_min").value)
        self.warning_interval_s = float(self.get_parameter("rtf_warning_interval_s").value)
        monitor_rate_hz = float(self.get_parameter("monitor_rate_hz").value)
        self.estimator = RtfEstimator(float(self.get_parameter("rtf_window_s").value))
        self.log = StructuredLog(
            "gazebo_monitor_node",
            str(self.get_parameter("log_file").value),
            self.get_logger(),
        )
        self.publisher = self.create_publisher(Float64, "/system/gazebo_rtf", 10)
        self.subscription = self.create_subscription(Clock, "/clock", self._clock_callback, 50)
        self.timer = self.create_timer(1.0 / monitor_rate_hz, self._evaluate)
        self.last_warning_monotonic = float("-inf")
        self.degraded = False
        self.log.event("INFO", "GAZEBO_MONITOR_STARTED", "Gazebo real-time-factor monitor started.")

    def _clock_callback(self, msg: Clock) -> None:
        sim_s = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9
        self.estimator.add(time.monotonic(), sim_s)

    def _evaluate(self) -> None:
        now = time.monotonic()
        estimate = self.estimator.estimate(now)
        if estimate is None:
            return

        value = Float64()
        value.data = estimate
        self.publisher.publish(value)

        if estimate < self.rtf_min:
            self.degraded = True
            if (now - self.last_warning_monotonic) >= self.warning_interval_s:
                self.last_warning_monotonic = now
                self.log.event(
                    "WARNING",
                    "GAZEBO_RTF_LOW",
                    (
                        f"Gazebo real-time factor is {estimate:.3f}, below the configured minimum "
                        f"of {self.rtf_min:.3f}; warning will repeat until recovery."
                    ),
                    real_time_factor=round(estimate, 6),
                )
        elif self.degraded:
            self.degraded = False
            self.log.event(
                "INFO",
                "GAZEBO_RTF_RECOVERED",
                f"Gazebo real-time factor recovered to {estimate:.3f}.",
                real_time_factor=round(estimate, 6),
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
