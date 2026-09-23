#!/usr/bin/env python3
import time

import rclpy
from hunav_msgs.msg import Agents
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool, Trigger


class EmptyHumanStates(Node):
    def __init__(self):
        super().__init__("empty_human_states")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(Agents, "/human_states", qos)
        self.enabled = True
        self.suppress_until_wall = 0.0
        self.create_service(SetBool, "~/set_enabled", self.set_enabled)
        self.create_service(Trigger, "~/publish_stale", self.publish_stale)
        self.create_service(Trigger, "~/publish_future", self.publish_future)
        self.timer = self.create_timer(0.025, self.publish_state)

    def publish_state(self):
        if not self.enabled or time.monotonic() < self.suppress_until_wall:
            return
        message = Agents()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        self.publisher.publish(message)

    def set_enabled(self, request, response):
        self.enabled = request.data
        self.suppress_until_wall = 0.0
        response.success = True
        response.message = "enabled" if self.enabled else "disabled"
        return response

    def publish_stale(self, request, response):
        del request
        message = Agents()
        message.header.stamp = (
            self.get_clock().now() - Duration(seconds=10.0)
        ).to_msg()
        message.header.frame_id = "map"
        self.publisher.publish(message)
        # Leave the stale sample visible across at least two 10 Hz controller
        # cycles before normal fresh publication resumes.
        self.suppress_until_wall = time.monotonic() + 0.25
        response.success = True
        response.message = "published one sample stamped 10 s in the past"
        return response

    def publish_future(self, request, response):
        del request
        message = Agents()
        message.header.stamp = (
            self.get_clock().now() + Duration(seconds=10.0)
        ).to_msg()
        message.header.frame_id = "map"
        self.publisher.publish(message)
        self.suppress_until_wall = time.monotonic() + 0.25
        response.success = True
        response.message = "published one sample stamped 10 s in the future"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = EmptyHumanStates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
