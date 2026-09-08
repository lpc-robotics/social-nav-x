#!/usr/bin/env python3
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from hunav_msgs.msg import Agent, Agents
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool


class SolverFailureProbe(Node):
    def __init__(self, timeout):
        super().__init__("arena_mpc_p2_solver_failure_probe")
        self.timeout = timeout
        self.navigation = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.set_enabled = self.create_client(
            SetBool, "/empty_human_states/set_enabled"
        )
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.human_publisher = self.create_publisher(Agents, "/human_states", qos)
        self.odom = None
        self.output = None
        self.overlap_active = False
        self.output_events = []
        self.controller_events = []
        self.watchdog_events = []
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_output, 10)
        self.create_subscription(String, "/FollowPath/status", self.on_controller, 10)
        self.create_subscription(
            String, "/mpc_command_watchdog/status", self.on_watchdog, 10
        )
        self.create_timer(0.025, self.publish_overlap)

    def on_odom(self, message):
        self.odom = message

    def on_output(self, message):
        self.output = message
        self.output_events.append(
            (time.monotonic(), message.linear.x, message.angular.z)
        )

    def on_controller(self, message):
        self.controller_events.append((time.monotonic(), message.data))

    def on_watchdog(self, message):
        self.watchdog_events.append((time.monotonic(), message.data))

    def publish_overlap(self):
        if not self.overlap_active or self.odom is None:
            return
        message = Agents()
        message.header.stamp = self.odom.header.stamp
        message.header.frame_id = "map"
        human = Agent()
        human.id = 9001
        human.type = Agent.PERSON
        human.name = "p2_forced_overlap"
        human.position = self.odom.pose.pose
        human.radius = 0.30
        message.agents.append(human)
        self.human_publisher.publish(message)

    def spin_until(self, predicate, timeout=None):
        deadline = time.monotonic() + (timeout or self.timeout)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return True
        return False

    def enable_empty_source(self, enabled):
        if not self.set_enabled.wait_for_service(timeout_sec=15.0):
            raise RuntimeError("empty human-state control service unavailable")
        request = SetBool.Request()
        request.data = enabled
        future = self.set_enabled.call_async(request)
        if not self.spin_until(future.done, 15.0):
            raise RuntimeError("empty human-state control call timed out")
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(f"empty human-state control failed: {response}")

    @staticmethod
    def is_zero(event):
        return abs(event[1]) < 0.001 and abs(event[2]) < 0.001

    def run(self):
        if not self.spin_until(
            lambda: self.odom is not None and self.output is not None, 30.0
        ):
            raise RuntimeError("timed out waiting for odom and /cmd_vel")
        if not self.navigation.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("navigate_to_pose server unavailable")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.odom.pose.pose.position.x + 1.2
        goal.pose.pose.position.y = self.odom.pose.pose.position.y
        goal.pose.pose.orientation.w = 1.0
        sent = self.navigation.send_goal_async(goal)
        if not self.spin_until(sent.done, 10.0):
            raise RuntimeError("goal acceptance timed out")
        handle = sent.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("goal rejected")
        result = handle.get_result_async()
        if not self.spin_until(
            lambda: self.output is not None and abs(self.output.linear.x) >= 0.05,
            30.0,
        ):
            raise RuntimeError("moving command not observed")

        self.enable_empty_source(False)
        injection_start = time.monotonic()
        self.overlap_active = True

        first_zero = None
        first_zero_ros = None

        def stopped():
            nonlocal first_zero, first_zero_ros
            for event in self.output_events:
                if event[0] >= injection_start and self.is_zero(event):
                    first_zero = event[0]
                    first_zero_ros = (
                        self.odom.header.stamp.sec
                        + self.odom.header.stamp.nanosec * 1.0e-9
                    )
                    return True
            return False

        if not self.spin_until(stopped, 2.0):
            raise RuntimeError("solver failure did not produce a zero command")
        if not self.spin_until(result.done, 5.0):
            raise RuntimeError("solver failure did not terminate FollowPath")
        action_status = result.result().status

        stopped_ros = None

        def physically_stopped():
            nonlocal stopped_ros
            if self.odom is None:
                return False
            speed = math.hypot(
                self.odom.twist.twist.linear.x, self.odom.twist.twist.linear.y
            )
            angular = abs(self.odom.twist.twist.angular.z)
            now_ros = (
                self.odom.header.stamp.sec
                + self.odom.header.stamp.nanosec * 1.0e-9
            )
            if speed < 0.02 and angular < 0.05:
                stopped_ros = now_ros
                return True
            return False

        if not self.spin_until(physically_stopped, 5.0):
            raise RuntimeError("robot did not physically stop after solver failure")
        hold_end = time.monotonic() + 0.3
        while rclpy.ok() and time.monotonic() < hold_end:
            rclpy.spin_once(self, timeout_sec=0.02)
        rebound = sum(
            1
            for event in self.output_events
            if first_zero <= event[0] <= hold_end and not self.is_zero(event)
        )
        failure_statuses = [
            text
            for stamp, text in self.controller_events
            if stamp >= injection_start and text.startswith("stop failure=")
        ]
        watchdog_stops = [
            text
            for stamp, text in self.watchdog_events
            if stamp >= injection_start and text.startswith("stop reason=")
        ]
        self.overlap_active = False
        self.enable_empty_source(True)
        physical_stop_delay = max(0.0, stopped_ros - first_zero_ros)
        return {
            "mode": "forced_overlap_solver_failure",
            "action_status": action_status,
            "first_zero_latency_s": first_zero - injection_start,
            "physical_stop_sim_time_s": physical_stop_delay,
            "nonzero_after_first_zero": rebound,
            "controller_failure_statuses": failure_statuses,
            "watchdog_stop_statuses": watchdog_stops,
            "pass": action_status == GoalStatus.STATUS_ABORTED
            and first_zero - injection_start <= 0.27
            and physical_stop_delay <= 0.6
            and rebound == 0
            and len(failure_statuses) >= 5,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--output")
    args = parser.parse_args()
    rclpy.init()
    probe = SolverFailureProbe(args.timeout)
    exit_code = 0
    try:
        report = probe.run()
        if not report["pass"]:
            exit_code = 2
    except Exception as error:
        probe.overlap_active = False
        try:
            probe.enable_empty_source(True)
        except Exception:
            pass
        report = {
            "mode": "forced_overlap_solver_failure",
            "error": str(error),
            "pass": False,
        }
        exit_code = 1
    rendered = json.dumps(report, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered + "\n", encoding="utf-8")
    probe.destroy_node()
    rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
