import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class RuntimeVerifier(Node):
    def __init__(self):
        super().__init__("arena_runtime_verifier")
        self.odom = None
        self.scan_count = 0
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "/lidar", self._on_scan, 10)
        self.navigation = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def _on_odom(self, msg):
        self.odom = msg

    def _on_scan(self, _msg):
        self.scan_count += 1

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def run(self):
        if not self.spin_until(lambda: self.odom is not None and self.scan_count >= 3, 90.0):
            raise RuntimeError("timed out waiting for /odom and /lidar")
        start_x = self.odom.pose.pose.position.x
        start_y = self.odom.pose.pose.position.y

        if not self.navigation.wait_for_server(timeout_sec=90.0):
            raise RuntimeError("timed out waiting for /navigate_to_pose")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = 5.0
        goal.pose.pose.position.y = 3.0
        goal.pose.pose.orientation.w = 1.0
        send_future = self.navigation.send_goal_async(goal)
        if not self.spin_until(send_future.done, 30.0):
            raise RuntimeError("navigation goal was not accepted in time")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("navigation goal was rejected")

        result_future = goal_handle.get_result_async()
        if not self.spin_until(result_future.done, 150.0):
            goal_handle.cancel_goal_async()
            raise RuntimeError("navigation goal timed out")
        wrapped_result = result_future.result()
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"navigation finished with status={wrapped_result.status}")

        end_x = self.odom.pose.pose.position.x
        end_y = self.odom.pose.pose.position.y
        distance = math.hypot(end_x - start_x, end_y - start_y)
        if distance < 1.0:
            raise RuntimeError(f"robot did not move far enough: {distance:.3f} m")
        print(
            f"SMOKE_NAVIGATION_OK start=({start_x:.3f},{start_y:.3f}) "
            f"end=({end_x:.3f},{end_y:.3f}) moved={distance:.3f}m "
            f"lidar_messages={self.scan_count}",
            flush=True,
        )


def main(args=None):
    rclpy.init(args=args)
    node = RuntimeVerifier()
    status = 0
    try:
        node.run()
    except Exception as exc:
        print(f"SMOKE_NAVIGATION_FAILED {exc}", file=sys.stderr, flush=True)
        status = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(status)
