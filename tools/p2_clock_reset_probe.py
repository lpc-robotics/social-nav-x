#!/usr/bin/env python3
import argparse
import json
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


class ClockResetProbe(Node):
    def __init__(self, timeout):
        super().__init__("arena_mpc_p2_clock_reset_probe")
        self.timeout = timeout
        self.navigation = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        clock_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.clock_publisher = self.create_publisher(Clock, "/clock", clock_qos)
        self.odom = None
        self.output = None
        self.latest_clock_ns = 0
        self.output_events = []
        self.watchdog_events = []
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_output, 10)
        self.create_subscription(Clock, "/clock", self.on_clock, clock_qos)
        self.create_subscription(
            String, "/mpc_command_watchdog/status", self.on_watchdog, 10
        )

    def on_odom(self, message):
        self.odom = message

    def on_output(self, message):
        self.output = message
        self.output_events.append(
            (time.monotonic(), message.linear.x, message.angular.z)
        )

    def on_clock(self, message):
        stamp = message.clock
        self.latest_clock_ns = max(
            self.latest_clock_ns, stamp.sec * 1_000_000_000 + stamp.nanosec
        )

    def on_watchdog(self, message):
        self.watchdog_events.append((time.monotonic(), message.data))

    def spin_until(self, predicate, timeout=None):
        deadline = time.monotonic() + (timeout or self.timeout)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return True
        return False

    @staticmethod
    def is_zero(event):
        return abs(event[1]) < 0.001 and abs(event[2]) < 0.001

    def run(self):
        if not self.spin_until(
            lambda: self.odom is not None
            and self.output is not None
            and self.latest_clock_ns > 5_000_000_000,
            30.0,
        ):
            raise RuntimeError("timed out waiting for odom, command, and clock")
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

        before_ns = self.latest_clock_ns
        rollback_ns = before_ns - 5_000_000_000
        rollback = Clock()
        rollback.clock = Time(
            sec=rollback_ns // 1_000_000_000,
            nanosec=rollback_ns % 1_000_000_000,
        )
        injection_start = time.monotonic()
        for _ in range(10):
            self.clock_publisher.publish(rollback)
            rclpy.spin_once(self, timeout_sec=0.02)

        reason_time = None
        first_zero = None

        def latch_observed():
            nonlocal reason_time, first_zero
            for stamp, text in self.watchdog_events:
                if stamp >= injection_start and text == "stop reason=clock_reset_latched":
                    reason_time = stamp
                    break
            for event in self.output_events:
                if event[0] >= injection_start and self.is_zero(event):
                    first_zero = event[0]
                    break
            return reason_time is not None and first_zero is not None

        if not self.spin_until(latch_observed, 2.0):
            raise RuntimeError("clock rollback did not latch watchdog stop")

        hold_end = time.monotonic() + 0.5
        while rclpy.ok() and time.monotonic() < hold_end:
            rclpy.spin_once(self, timeout_sec=0.02)
        rebound = sum(
            1
            for event in self.output_events
            if first_zero <= event[0] <= hold_end and not self.is_zero(event)
        )
        cancel = handle.cancel_goal_async()
        self.spin_until(cancel.done, 5.0)
        self.spin_until(result.done, 5.0)
        action_status = (
            result.result().status if result.done() else GoalStatus.STATUS_UNKNOWN
        )
        return {
            "mode": "clock_rollback",
            "clock_before_ns": before_ns,
            "injected_clock_ns": rollback_ns,
            "rollback_ns": before_ns - rollback_ns,
            "watchdog_reason_latency_s": reason_time - injection_start,
            "first_zero_latency_s": first_zero - injection_start,
            "nonzero_after_latch": rebound,
            "action_status": action_status,
            "restart_required": True,
            "pass": first_zero - injection_start <= 0.27 and rebound == 0,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--output")
    args = parser.parse_args()
    rclpy.init()
    probe = ClockResetProbe(args.timeout)
    exit_code = 0
    try:
        report = probe.run()
        if not report["pass"]:
            exit_code = 2
    except Exception as error:
        report = {"mode": "clock_rollback", "error": str(error), "pass": False}
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
