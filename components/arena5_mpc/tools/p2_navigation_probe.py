#!/usr/bin/env python3
import argparse
import json
import math
import re
import statistics
import sys
import time
from collections import Counter
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from hunav_msgs.msg import Agents
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


TIMING = re.compile(r"solve_ms=([0-9.]+) cycle_ms=([0-9.]+)")


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class Probe(Node):
    def __init__(self, goal_x, goal_y, timeout):
        super().__init__("arena_mpc_p2_navigation_probe")
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.timeout = timeout
        self.odom = None
        self.humans = None
        self.start = None
        self.start_odom_stamp = None
        self.run_started_wall = None
        self.solve_ms = []
        self.cycle_ms = []
        self.timed_cycles = []
        self.controller_status = []
        self.watchdog_status = []
        self.output_nonzero = 0
        self.output_total = 0
        self.raw_linear = []
        self.output_linear = []
        self.raw_times = []
        self.output_times = []
        self.minimum_clearance = math.inf
        self.navigation = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Agents, "/human_states", self.on_humans, 10)
        self.create_subscription(String, "/FollowPath/status", self.on_controller_status, 10)
        self.create_subscription(
            String, "/mpc_command_watchdog/status", self.on_watchdog_status, 10
        )
        self.create_subscription(Twist, "/cmd_vel_nav", self.on_raw, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_output, 10)

    def on_odom(self, message):
        self.odom = message
        self.update_clearance()

    def on_humans(self, message):
        self.humans = message
        self.update_clearance()

    def on_controller_status(self, message):
        now = time.monotonic()
        self.controller_status.append((now, message.data))
        match = TIMING.search(message.data)
        if match:
            self.solve_ms.append(float(match.group(1)))
            cycle = float(match.group(2))
            self.cycle_ms.append(cycle)
            self.timed_cycles.append((now, cycle))

    def on_watchdog_status(self, message):
        self.watchdog_status.append((time.monotonic(), message.data))

    def on_raw(self, message):
        self.raw_times.append(time.monotonic())
        self.raw_linear.append(message.linear.x)

    def on_output(self, message):
        self.output_total += 1
        self.output_times.append(time.monotonic())
        self.output_linear.append(message.linear.x)
        if abs(message.linear.x) >= 0.001 or abs(message.angular.z) >= 0.001:
            self.output_nonzero += 1

    def update_clearance(self):
        if self.odom is None or self.humans is None:
            return
        robot = self.odom.pose.pose
        yaw = yaw_from_quaternion(robot.orientation)
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        for human in self.humans.agents:
            dx_world = human.position.position.x - robot.position.x
            dy_world = human.position.position.y - robot.position.y
            x = cosine * dx_world + sine * dy_world
            y = -sine * dx_world + cosine * dy_world
            outside_x = max(abs(x) - 0.24, 0.0)
            outside_y = max(abs(y) - 0.22, 0.0)
            if outside_x > 0.0 or outside_y > 0.0:
                point_distance = math.hypot(outside_x, outside_y)
            else:
                point_distance = -min(0.24 - abs(x), 0.22 - abs(y))
            self.minimum_clearance = min(
                self.minimum_clearance, point_distance - human.radius
            )

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def run(self):
        if not self.spin_until(lambda: self.odom is not None and self.humans is not None, 30.0):
            raise RuntimeError("timed out waiting for odom and HuNav")
        self.start = (
            self.odom.pose.pose.position.x,
            self.odom.pose.pose.position.y,
        )
        self.start_odom_stamp = (
            self.odom.header.stamp.sec + 1.0e-9 * self.odom.header.stamp.nanosec
        )
        self.run_started_wall = time.monotonic()
        if not self.navigation.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("navigate_to_pose server unavailable")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.goal_x
        goal.pose.pose.position.y = self.goal_y
        goal.pose.pose.orientation.w = 1.0
        sent = self.navigation.send_goal_async(goal)
        if not self.spin_until(sent.done, 10.0):
            raise RuntimeError("goal acceptance timed out")
        handle = sent.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("goal rejected")
        result = handle.get_result_async()
        if not self.spin_until(result.done, self.timeout):
            handle.cancel_goal_async()
            raise RuntimeError("navigation timed out")
        return result.result().status

    def report(self, status, error=None):
        end = None
        end_odom_stamp = None
        if self.odom is not None:
            end = (self.odom.pose.pose.position.x, self.odom.pose.pose.position.y)
            end_odom_stamp = (
                self.odom.header.stamp.sec + 1.0e-9 * self.odom.header.stamp.nanosec
            )
        raw_intervals = [b - a for a, b in zip(self.raw_times, self.raw_times[1:])]
        output_intervals = [b - a for a, b in zip(self.output_times, self.output_times[1:])]
        controller_reasons = Counter(
            text.split(" reason=", 1)[1].split(" solve_ms=", 1)[0]
            for _, text in self.controller_status
            if text.startswith("stop ") and " reason=" in text
        )
        watchdog_reasons = Counter(
            text.removeprefix("stop reason=")
            for _, text in self.watchdog_status
            if text.startswith("stop reason=")
        )
        status_to_raw_ms = []
        full_command_ms = []
        raw_index = 0
        for status_index, (status_time, cycle_ms) in enumerate(self.timed_cycles):
            next_status_time = (
                self.timed_cycles[status_index + 1][0]
                if status_index + 1 < len(self.timed_cycles)
                else math.inf
            )
            while raw_index < len(self.raw_times) and self.raw_times[raw_index] < status_time:
                raw_index += 1
            if raw_index >= len(self.raw_times) or self.raw_times[raw_index] >= next_status_time:
                continue
            boundary_ms = 1000.0 * (self.raw_times[raw_index] - status_time)
            status_to_raw_ms.append(boundary_ms)
            full_command_ms.append(cycle_ms + boundary_ms)
            raw_index += 1
        displacement = (
            math.dist(self.start, end) if self.start is not None and end is not None else None
        )
        final_goal_error = (
            math.dist((self.goal_x, self.goal_y), end) if end is not None else None
        )
        full_p99 = percentile(full_command_ms, 0.99)
        full_over_fraction = (
            sum(value > 100.0 for value in full_command_ms) / len(full_command_ms)
            if full_command_ms
            else None
        )
        passed = (
            status == GoalStatus.STATUS_SUCCEEDED
            and displacement is not None
            and displacement >= 0.5
            and bool(full_command_ms)
            and full_p99 <= 100.0
            and full_over_fraction <= 0.01
        )
        return {
            "goal": [self.goal_x, self.goal_y],
            "action_status": status,
            "succeeded": status == GoalStatus.STATUS_SUCCEEDED,
            "error": error,
            "start": list(self.start) if self.start is not None else None,
            "end": list(end) if end is not None else None,
            "displacement_m": displacement,
            "final_goal_error_m": final_goal_error,
            "elapsed_wall_s": (
                time.monotonic() - self.run_started_wall
                if self.run_started_wall is not None
                else None
            ),
            "elapsed_odom_stamp_s": (
                end_odom_stamp - self.start_odom_stamp
                if end_odom_stamp is not None and self.start_odom_stamp is not None
                else None
            ),
            "minimum_sampled_footprint_human_clearance_m": (
                self.minimum_clearance if math.isfinite(self.minimum_clearance) else None
            ),
            "successful_solves": len(self.solve_ms),
            "solver_ms_p50": statistics.median(self.solve_ms) if self.solve_ms else None,
            "solver_ms_p95": percentile(self.solve_ms, 0.95),
            "solver_ms_max": max(self.solve_ms) if self.solve_ms else None,
            "cycle_ms_p95": percentile(self.cycle_ms, 0.95),
            "cycle_ms_max": max(self.cycle_ms) if self.cycle_ms else None,
            "cross_boundary_samples": len(status_to_raw_ms),
            "status_to_raw_ms_p95": percentile(status_to_raw_ms, 0.95),
            "status_to_raw_ms_max": max(status_to_raw_ms, default=None),
            "full_command_ms_p95": percentile(full_command_ms, 0.95),
            "full_command_ms_p99": full_p99,
            "full_command_ms_max": max(full_command_ms, default=None),
            "full_command_over_100ms_fraction": full_over_fraction,
            "raw_interval_p95_s": percentile(raw_intervals, 0.95),
            "output_interval_p95_s": percentile(output_intervals, 0.95),
            "output_nonzero_fraction": (
                self.output_nonzero / self.output_total if self.output_total else None
            ),
            "raw_linear_max": max(self.raw_linear, default=None),
            "output_linear_max": max(self.output_linear, default=None),
            "controller_failure_counts": dict(controller_reasons),
            "watchdog_stop_counts": dict(watchdog_reasons),
            "controller_status_tail": [text for _, text in self.controller_status[-12:]],
            "watchdog_status_tail": [text for _, text in self.watchdog_status[-12:]],
            "pass": passed,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal-x", type=float, required=True)
    parser.add_argument("--goal-y", type=float, required=True)
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--output")
    args = parser.parse_args()
    rclpy.init()
    probe = Probe(args.goal_x, args.goal_y, args.timeout)
    exit_code = 0
    error_text = None
    try:
        status = probe.run()
    except Exception as error:
        status = GoalStatus.STATUS_UNKNOWN
        exit_code = 1
        error_text = str(error)
        print(f"P2_NAVIGATION_PROBE_ERROR {error}", file=sys.stderr)
    report = probe.report(status, error_text)
    if status != GoalStatus.STATUS_UNKNOWN and not report["pass"]:
        exit_code = 2
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
