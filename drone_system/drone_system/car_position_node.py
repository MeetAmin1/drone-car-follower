from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter

from .structured_logging import StructuredLog


class CarPositionNode(Node):
    """Relays the car's observed odometry to the required /car/position topic.

    The source is a dedicated Gazebo odometry publisher attached to the car model.
    This node does not know the scripted trajectory, query Gazebo services, or read
    model-state parameters.
    """

    def __init__(self) -> None:
        super().__init__("car_position_node")
        self.declare_parameter("source_topic", Parameter.Type.STRING)
        self.declare_parameter("publish_topic", Parameter.Type.STRING)
        self.declare_parameter("log_file", "")

        source_topic = str(self.get_parameter("source_topic").value)
        publish_topic = str(self.get_parameter("publish_topic").value)
        self.log = StructuredLog(
            "car_position_node",
            str(self.get_parameter("log_file").value),
            self.get_logger(),
        )

        self.publisher = self.create_publisher(PoseStamped, publish_topic, 20)
        self.subscription = self.create_subscription(Odometry, source_topic, self._callback, 20)
        self.seen_once = False
        self.log.event(
            "INFO",
            "CAR_POSITION_RELAY_STARTED",
            "Car position relay started and is waiting for the car odometry stream.",
        )

    def _callback(self, msg: Odometry) -> None:
        out = PoseStamped()
        out.header = msg.header
        out.header.frame_id = "world"
        out.pose = msg.pose.pose
        self.publisher.publish(out)

        if not self.seen_once:
            self.seen_once = True
            self.log.event(
                "INFO",
                "CAR_POSITION_OBSERVED",
                "Observed car odometry and began publishing /car/position.",
                source_frame=msg.header.frame_id,
                child_frame=msg.child_frame_id,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CarPositionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
