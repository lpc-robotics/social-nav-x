import math
import time

import rclpy
from geometry_msgs.msg import Twist
from hunav_msgs.msg import Agent, Agents
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


EXPECTED = {
    "regular": 1,
    "impassive": 2,
    "surprised": 3,
    "scared": 4,
    "curious": 5,
    "threatening": 6,
}


class SixBehaviorsVerifier(Node):
    def __init__(self):
        super().__init__("verify_six_behaviors")
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Agents, "/human_states", self._on_agents, 10)
        self.create_subscription(Agent, "/robot_states", self._on_robot, 10)
        self.create_subscription(
            Odometry, "/odom", self._on_odom, qos_profile_sensor_data
        )
        self.agent_types = {}
        self.active_types = set()
        self.response_types = set()
        self.agent_positions = {}
        self.robot_states = 0
        self.robot = None
        self.odom_xy = None

    def _on_agents(self, msg: Agents):
        for agent in msg.agents:
            self.agent_types[agent.name] = int(agent.behavior.type)
            self.agent_positions.setdefault(
                agent.name, (agent.position.position.x, agent.position.position.y)
            )
            if agent.behavior.state != 0:
                self.active_types.add(int(agent.behavior.type))
                self.response_types.add(int(agent.behavior.type))

            # HuNav v1's ScaredNav and ThreateningNav actions call
            # computeForces(), which resets behavior.state to zero before the
            # response is serialized. Verify those two actions by their
            # official kinematic outcome instead of relying on that transient
            # state bit.
            if self.robot is not None and agent.behavior.type == 4:
                dx = agent.position.position.x - self.robot.position.position.x
                dy = agent.position.position.y - self.robot.position.position.y
                radial_velocity = agent.velocity.linear.x * dx + agent.velocity.linear.y * dy
                if math.hypot(dx, dy) <= 3.5 and radial_velocity > 0.05:
                    self.response_types.add(4)
            if self.robot is not None and agent.behavior.type == 6:
                target_x = self.robot.position.position.x + agent.behavior.dist * math.cos(
                    self.robot.yaw
                )
                target_y = self.robot.position.position.y + agent.behavior.dist * math.sin(
                    self.robot.yaw
                )
                target_distance = math.hypot(
                    agent.position.position.x - target_x,
                    agent.position.position.y - target_y,
                )
                if target_distance <= 0.8:
                    self.response_types.add(6)

    def _on_robot(self, msg: Agent):
        self.robot = msg
        self.robot_states += 1

    def _on_odom(self, msg: Odometry):
        self.odom_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def publish_command(self, linear: float, angular: float = 0.0):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd_pub.publish(msg)


def _spin_until(node, predicate, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def main(args=None):
    rclpy.init(args=args)
    node = SixBehaviorsVerifier()
    try:
        ready = _spin_until(
            node,
            lambda: node.agent_types == EXPECTED
            and node.robot_states > 2
            and node.odom_xy is not None,
            45.0,
        )
        if not ready:
            raise RuntimeError(
                f"six behavior topics not ready: types={node.agent_types}, "
                f"robot_states={node.robot_states}, odom={node.odom_xy}"
            )

        start = node.odom_xy
        deadline = time.monotonic() + 6.0
        while rclpy.ok() and time.monotonic() < deadline:
            node.publish_command(0.45)
            rclpy.spin_once(node, timeout_sec=0.05)
        for _ in range(10):
            node.publish_command(0.0)
            rclpy.spin_once(node, timeout_sec=0.05)

        # HuNav computation and Isaac display are intentionally asynchronous.
        # Keep the robot stopped while allowing late special-behavior states to
        # arrive instead of judging immediately after the motion command.
        response_deadline = time.monotonic() + 45.0
        while (
            rclpy.ok()
            and time.monotonic() < response_deadline
            and not {3, 4, 5, 6}.issubset(node.response_types)
        ):
            node.publish_command(0.0)
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.odom_xy is None:
            raise RuntimeError("odometry disappeared during motion test")
        distance = math.hypot(node.odom_xy[0] - start[0], node.odom_xy[1] - start[1])
        missing_responses = {3, 4, 5, 6} - node.response_types
        if distance < 0.5:
            raise RuntimeError(f"robot did not move far enough: {distance:.3f} m")
        if missing_responses:
            raise RuntimeError(
                "special HuNav behavior responses were not observed: "
                f"{sorted(missing_responses)}"
            )
        print(
            "SIX_BEHAVIORS_VERIFY_OK "
            f"types={','.join(str(EXPECTED[name]) for name in EXPECTED)} "
            f"active={','.join(str(value) for value in sorted(node.active_types))} "
            f"responses={','.join(str(value) for value in sorted(node.response_types))} "
            f"robot_distance={distance:.3f} robot_states={node.robot_states}"
        )
    finally:
        node.publish_command(0.0)
        node.destroy_node()
        rclpy.shutdown()
