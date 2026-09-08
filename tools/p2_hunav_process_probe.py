#!/usr/bin/env python3
import argparse
import json
import os
import signal
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
from hunav_msgs.msg import Agents
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


class HuNavProcessProbe(Node):
    def __init__(self, bridge_pid, timeout, pause_seconds):
        super().__init__("arena_mpc_p2_hunav_process_probe")
        self.bridge_pid = bridge_pid
        self.timeout = timeout
        self.pause_seconds = pause_seconds
        self.navigation = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.odom = None
        self.clock_ns = None
        self.outputs = []
        self.humans = []
        self.watchdog = []
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_output, 10)
        self.create_subscription(Agents, "/human_states", self.on_humans, 10)
        self.create_subscription(
            String, "/mpc_command_watchdog/status", self.on_watchdog, 10
        )

    def on_odom(self, message):
        self.odom = message

    def on_clock(self, message):
        self.clock_ns = message.clock.sec * 1_000_000_000 + message.clock.nanosec

    def on_output(self, message):
        self.outputs.append(
            (time.monotonic(), message.linear.x, message.angular.z)
        )

    def on_humans(self, message):
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        now_ns = self.clock_ns if self.clock_ns is not None else 0
        self.humans.append((time.monotonic(), stamp_ns, now_ns, len(message.agents)))

    def on_watchdog(self, message):
        self.watchdog.append((time.monotonic(), message.data))

    def spin_until(self, predicate, timeout=None):
        deadline = time.monotonic() + (timeout or self.timeout)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return True
        return False

    @staticmethod
    def zero(event):
        return abs(event[1]) < 0.001 and abs(event[2]) < 0.001

    def validate_pid(self):
        cmdline = Path(f"/proc/{self.bridge_pid}/cmdline").read_bytes().replace(b"\0", b" ")
        if b"hunav_six_behaviors_bridge" not in cmdline:
            raise RuntimeError(
                f"pid {self.bridge_pid} is not hunav_six_behaviors_bridge: "
                + cmdline.decode(errors="replace")
            )

    def run(self):
        self.validate_pid()
        if not self.spin_until(
            lambda: self.odom is not None
            and self.clock_ns is not None
            and len(self.humans) >= 5
            and len(self.outputs) >= 2,
            30.0,
        ):
            raise RuntimeError("timed out waiting for odom, human states, and output")
        if not self.navigation.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("navigate_to_pose server unavailable")

        start = self.odom.pose.pose.position
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = start.x - 0.7
        goal.pose.pose.position.y = start.y
        goal.pose.pose.orientation.w = 1.0
        sent = self.navigation.send_goal_async(goal)
        if not self.spin_until(sent.done, 30.0):
            raise RuntimeError("goal acceptance timed out")
        handle = sent.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("goal rejected")
        result = handle.get_result_async()
        if not self.spin_until(
            lambda: any(
                abs(item[1]) >= 0.02 or abs(item[2]) >= 0.05
                for item in self.outputs[-30:]
            ),
            20.0,
        ):
            raise RuntimeError("moving command not observed")

        latest_before_pause = self.humans[-1]
        pause_start = time.monotonic()
        os.kill(self.bridge_pid, signal.SIGSTOP)
        resumed = False
        try:
            first_zero = None
            reason_time = None

            def outage_seen():
                nonlocal first_zero, reason_time
                for event in self.outputs:
                    if event[0] >= pause_start and self.zero(event):
                        first_zero = event[0]
                        break
                for stamp, text in self.watchdog:
                    if stamp >= pause_start and text == "stop reason=human_input":
                        reason_time = stamp
                        break
                return first_zero is not None and reason_time is not None

            if not self.spin_until(outage_seen, self.pause_seconds + 0.5):
                raise RuntimeError("bridge pause did not trigger human_input watchdog stop")

            target_resume = pause_start + self.pause_seconds
            while time.monotonic() < target_resume:
                rclpy.spin_once(self, timeout_sec=0.02)
            resume_time = time.monotonic()
            nonzero_while_paused = sum(
                1
                for item in self.outputs
                if first_zero <= item[0] <= resume_time and not self.zero(item)
            )
            os.kill(self.bridge_pid, signal.SIGCONT)
            resumed = True
            if not self.spin_until(
                lambda: any(item[0] >= resume_time for item in self.humans), 5.0
            ):
                raise RuntimeError("human stream did not resume")
            first_after_resume = next(item for item in self.humans if item[0] >= resume_time)

            first_nonzero_after_resume = None

            def output_recovered():
                nonlocal first_nonzero_after_resume
                for item in self.outputs:
                    if item[0] >= resume_time and not self.zero(item):
                        first_nonzero_after_resume = item[0]
                        return True
                return False

            recovered = self.spin_until(output_recovered, 3.0)
            cancel = handle.cancel_goal_async()
            self.spin_until(cancel.done, 5.0)
            self.spin_until(result.done, 5.0)
            action_status = result.result().status if result.done() else None
        finally:
            if not resumed:
                try:
                    os.kill(self.bridge_pid, signal.SIGCONT)
                except ProcessLookupError:
                    pass

        first_age = (first_after_resume[2] - first_after_resume[1]) * 1e-9
        first_step = (first_after_resume[1] - latest_before_pause[1]) * 1e-9
        return {
            "bridge_pid": self.bridge_pid,
            "pause_requested_s": self.pause_seconds,
            "first_zero_from_pause_s": first_zero - pause_start,
            "watchdog_reason_from_pause_s": reason_time - pause_start,
            "nonzero_while_paused_after_first_zero": nonzero_while_paused,
            "first_human_after_resume_wall_s": first_after_resume[0] - resume_time,
            "first_human_after_resume_ros_age_s": first_age,
            "first_human_stamp_step_s": first_step,
            "first_human_agent_count": first_after_resume[3],
            "output_recovered": recovered,
            "first_nonzero_after_resume_s": None
            if first_nonzero_after_resume is None
            else first_nonzero_after_resume - resume_time,
            "first_nonzero_after_first_human_s": None
            if first_nonzero_after_resume is None
            else first_nonzero_after_resume - first_after_resume[0],
            "action_status": action_status,
            "pass": first_zero - pause_start <= 0.65
            and nonzero_while_paused == 0
            and first_after_resume[3] == 6
            and first_age <= 0.3
            and recovered,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bridge-pid", type=int, required=True)
    parser.add_argument("--pause-seconds", type=float, default=0.75)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--output")
    args = parser.parse_args()
    rclpy.init()
    probe = HuNavProcessProbe(args.bridge_pid, args.timeout, args.pause_seconds)
    exit_code = 0
    try:
        report = probe.run()
        if not report["pass"]:
            exit_code = 2
    except Exception as error:
        report = {"error": str(error), "pass": False}
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
