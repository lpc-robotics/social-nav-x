#!/usr/bin/env python3
import argparse
import json
import math
import re
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


EPOCH = re.compile(r"epoch=([0-9]+)")


class HumanFaultProbe(Node):
    def __init__(self, timeout):
        super().__init__("arena_mpc_p2_human_fault_probe")
        self.timeout = timeout
        self.navigation = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.set_enabled = self.create_client(
            SetBool, "/empty_human_states/set_enabled"
        )
        self.publish_stale = self.create_client(
            Trigger, "/empty_human_states/publish_stale"
        )
        self.publish_future = self.create_client(
            Trigger, "/empty_human_states/publish_future"
        )
        self.odom = None
        self.output = None
        self.output_events = []
        self.watchdog_events = []
        self.controller_events = []
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_output, 10)
        self.create_subscription(
            String, "/mpc_command_watchdog/status", self.on_watchdog, 10
        )
        self.create_subscription(String, "/FollowPath/status", self.on_controller, 10)

    def on_odom(self, message):
        self.odom = message

    def on_output(self, message):
        self.output = message
        self.output_events.append(
            (time.monotonic(), message.linear.x, message.angular.z)
        )

    def on_watchdog(self, message):
        self.watchdog_events.append((time.monotonic(), message.data))

    def on_controller(self, message):
        self.controller_events.append((time.monotonic(), message.data))

    def spin_until(self, predicate, timeout=None):
        deadline = time.monotonic() + (timeout or self.timeout)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return True
        return False

    def wait_ready(self):
        if not self.spin_until(
            lambda: self.odom is not None and self.output is not None, 30.0
        ):
            raise RuntimeError("timed out waiting for odom and /cmd_vel")
        if not self.navigation.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("navigate_to_pose server unavailable")

    def send_goal(self, distance=1.2):
        start = self.odom.pose.pose.position
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = start.x + distance
        goal.pose.pose.position.y = start.y
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
        return handle, result

    def call_set_enabled(self, enabled):
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
        return time.monotonic(), response.message

    def call_bad_stamp(self, mode):
        client = self.publish_stale if mode == "stale" else self.publish_future
        if not client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError(f"{mode} human-state service unavailable")
        future = client.call_async(Trigger.Request())
        if not self.spin_until(future.done, 15.0):
            raise RuntimeError(f"{mode} human-state call timed out")
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(f"{mode} human-state call failed: {response}")
        return time.monotonic(), response.message

    @staticmethod
    def is_zero(event):
        return abs(event[1]) < 0.001 and abs(event[2]) < 0.001

    def first_zero_after(self, start):
        for event in self.output_events:
            if event[0] >= start and self.is_zero(event):
                return event[0]
        return None

    def stop_reason_after(self, start, reason):
        expected = f"stop reason={reason}"
        for stamp, text in self.watchdog_events:
            if stamp >= start and text == expected:
                return stamp
        return None

    def outage(self):
        self.wait_ready()
        handle, result = self.send_goal()
        request_time = time.monotonic()
        disabled_ack, disabled_message = self.call_set_enabled(False)

        first_zero = None
        reason_time = None

        def stopped_for_human_input():
            nonlocal first_zero, reason_time
            first_zero = self.first_zero_after(request_time)
            reason_time = self.stop_reason_after(request_time, "human_input")
            return first_zero is not None and reason_time is not None

        if not self.spin_until(stopped_for_human_input, 2.0):
            self.call_set_enabled(True)
            raise RuntimeError("human outage did not produce watchdog zero/human_input")

        hold_end = time.monotonic() + 0.15
        while rclpy.ok() and time.monotonic() < hold_end:
            rclpy.spin_once(self, timeout_sec=0.02)
        nonzero_while_disabled = sum(
            1
            for event in self.output_events
            if first_zero <= event[0] <= hold_end and not self.is_zero(event)
        )
        enabled_ack, enabled_message = self.call_set_enabled(True)
        if not self.spin_until(result.done):
            cancel = handle.cancel_goal_async()
            self.spin_until(cancel.done, 5.0)
            raise RuntimeError("navigation did not recover after human stream resumed")
        action_status = result.result().status
        return {
            "mode": "outage",
            "disable_response": disabled_message,
            "enable_response": enabled_message,
            "disable_ack_latency_s": disabled_ack - request_time,
            "first_zero_from_disable_request_s": first_zero - request_time,
            "first_zero_from_disable_ack_s": first_zero - disabled_ack,
            "watchdog_reason_latency_s": reason_time - request_time,
            "nonzero_while_disabled_after_first_zero": nonzero_while_disabled,
            "resume_ack_s": enabled_ack - request_time,
            "action_status": action_status,
            "pass": first_zero - disabled_ack <= 0.62
            and nonzero_while_disabled == 0
            and action_status == GoalStatus.STATUS_SUCCEEDED,
        }

    def bad_stamp(self, mode):
        self.wait_ready()
        _, result = self.send_goal(0.9)

        def latest_epoch():
            values = []
            for _, text in self.controller_events:
                match = EPOCH.search(text)
                if match:
                    values.append(int(match.group(1)))
            return max(values) if values else None

        if not self.spin_until(lambda: latest_epoch() is not None, 10.0):
            raise RuntimeError("no controller epoch observed")
        before_epoch = latest_epoch()
        request_time = time.monotonic()
        ack_time, service_message = self.call_bad_stamp(mode)

        def epoch_advanced():
            current = latest_epoch()
            return current is not None and current > before_epoch

        if not self.spin_until(epoch_advanced, 3.0):
            raise RuntimeError(f"{mode} human sample did not advance reset epoch")
        after_epoch = latest_epoch()
        first_zero = self.first_zero_after(request_time)
        if not self.spin_until(result.done):
            raise RuntimeError(f"navigation did not finish after {mode} human sample")
        action_status = result.result().status
        return {
            "mode": mode,
            "service_response": service_message,
            "service_ack_latency_s": ack_time - request_time,
            "epoch_before": before_epoch,
            "epoch_after": after_epoch,
            "first_zero_observed": first_zero is not None,
            "first_zero_latency_s": None
            if first_zero is None
            else first_zero - request_time,
            "action_status": action_status,
            "pass": after_epoch > before_epoch
            and first_zero is not None
            and action_status == GoalStatus.STATUS_SUCCEEDED,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("outage", "stale", "future"))
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--output")
    args = parser.parse_args()
    rclpy.init()
    probe = HumanFaultProbe(args.timeout)
    exit_code = 0
    try:
        report = probe.outage() if args.mode == "outage" else probe.bad_stamp(args.mode)
        if not report["pass"]:
            exit_code = 2
    except Exception as error:
        report = {"mode": args.mode, "error": str(error), "pass": False}
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
