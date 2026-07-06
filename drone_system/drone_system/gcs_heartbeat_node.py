from __future__ import annotations

import rclpy
from pymavlink import mavutil
from rclpy.node import Node
from rclpy.parameter import Parameter

from .structured_logging import StructuredLog


class GcsHeartbeatNode(Node):
    """Provides a minimal GCS heartbeat so headless PX4 SITL can arm safely."""

    def __init__(self) -> None:
        super().__init__("gcs_heartbeat_node")
        self.declare_parameter("px4_mavlink_endpoint", Parameter.Type.STRING)
        self.declare_parameter("heartbeat_rate_hz", Parameter.Type.DOUBLE)
        self.declare_parameter("log_file", "")

        endpoint = str(self.get_parameter("px4_mavlink_endpoint").value)
        rate_hz = float(self.get_parameter("heartbeat_rate_hz").value)
        self.log = StructuredLog(
            "gcs_heartbeat_node",
            str(self.get_parameter("log_file").value),
            self.get_logger(),
        )
        self.connection = mavutil.mavlink_connection(
            endpoint,
            source_system=255,
            source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
        )
        self.timer = self.create_timer(1.0 / rate_hz, self._send)
        self.sent_once = False

    def _send(self) -> None:
        self.connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )
        if not self.sent_once:
            self.sent_once = True
            self.log.event(
                "INFO",
                "GCS_HEARTBEAT_STARTED",
                "Headless GCS heartbeat is being sent to PX4 SITL.",
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GcsHeartbeatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.connection.close()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
