"""Convert final planar commands to namespaced Jackal wheel display commands."""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


WHEELS = [
    "front_left_wheel_joint",
    "rear_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_right_wheel_joint",
]


class WheelVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("wheel_visualizer")
        self.declare_parameter("robot_name", "")
        self.declare_parameter("wheel_radius", 0.098)
        self.declare_parameter("wheel_separation", 0.36)
        robot = str(self.get_parameter("robot_name").value).strip("/")
        if not robot:
            raise RuntimeError("robot_name is required")
        self.radius = float(self.get_parameter("wheel_radius").value)
        self.separation = float(self.get_parameter("wheel_separation").value)
        self.publisher = self.create_publisher(
            JointState, f"/{robot}/isaac/joint_commands_velocity", 10
        )
        self.create_subscription(Twist, f"/{robot}/cmd_vel", self._command, 10)

    def _command(self, command: Twist) -> None:
        left = (command.linear.x - command.angular.z * self.separation / 2.0) / self.radius
        right = (command.linear.x + command.angular.z * self.separation / 2.0) / self.radius
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = WHEELS
        message.velocity = [left, left, right, right]
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheelVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

