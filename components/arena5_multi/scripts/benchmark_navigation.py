#!/usr/bin/env python3
"""Run paired namespaced Nav2 goals and save action/geometry evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import subprocess
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/arena_multi_control"))
from arena_multi_control.scenario import load_scenario  # noqa: E402


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_error(a: float, b: float) -> float:
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)


class Benchmark(Node):
    def __init__(self, names):
        super().__init__("multirobot_navigation_benchmark")
        self.poses = {}
        self.distance = {name: 0.0 for name in names}
        self.minimum_separation = math.inf
        self.command_samples = {name: 0 for name in names}
        self.zero_command_samples = {name: 0 for name in names}
        self.action_clients = {name: ActionClient(self, NavigateToPose, f"/{name}/navigate_to_pose") for name in names}
        self._benchmark_subscriptions = [self.create_subscription(
            Odometry, f"/{name}/odom", lambda msg, n=name: self.on_odom(n, msg), qos_profile_sensor_data)
            for name in names]
        self._benchmark_subscriptions.extend(self.create_subscription(
            Twist, f"/{name}/cmd_vel", lambda msg, n=name: self.on_command(n, msg), 10)
            for name in names)

    def on_odom(self, name, message):
        current = (message.pose.pose.position.x, message.pose.pose.position.y, yaw_of(message.pose.pose.orientation))
        previous = self.poses.get(name)
        if previous is not None:
            self.distance[name] += math.hypot(current[0] - previous[0], current[1] - previous[1])
        self.poses[name] = current
        values = list(self.poses.values())
        for index, first in enumerate(values):
            for second in values[index + 1:]:
                self.minimum_separation = min(
                    self.minimum_separation, math.hypot(first[0] - second[0], first[1] - second[1]))

    def on_command(self, name, message):
        self.command_samples[name] += 1
        if abs(message.linear.x) < 1e-4 and abs(message.angular.z) < 1e-4:
            self.zero_command_samples[name] += 1


def goal_message(node, values):
    x, y, yaw = [float(item) for item in values]
    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return goal, (x, y, yaw)


def spin_until(node, futures, timeout):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline and not all(future.done() for future in futures):
        rclpy.spin_once(node, timeout_sec=0.05)
    return all(future.done() for future in futures)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario")
    parser.add_argument("tasks")
    parser.add_argument("--goal-timeout", type=float, default=600.0)
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    scenario_path = Path(args.scenario).resolve()
    tasks_path = Path(args.tasks).resolve()
    scenario = load_scenario(scenario_path)
    names = [robot.name for robot in scenario.robots if robot.control_mode == "nav2"]
    rounds = yaml.safe_load(tasks_path.read_text())["rounds"]
    if any(set(item) != set(names) for item in rounds):
        raise SystemExit("each task round must define exactly every Nav2 robot")

    rclpy.init()
    node = Benchmark(names)
    for name, client in node.action_clients.items():
        if not client.wait_for_server(timeout_sec=30.0):
            raise SystemExit(f"{name} action server unavailable")
    results = []
    failures = []
    for round_index, task in enumerate(rounds, start=1):
        before = dict(node.distance)
        commands_before = dict(node.command_samples)
        zero_before = dict(node.zero_command_samples)
        send_futures = []
        targets = {}
        for name in names:
            message, targets[name] = goal_message(node, task[name])
            send_futures.append(node.action_clients[name].send_goal_async(message))
        if not spin_until(node, send_futures, 30.0):
            failures.append(f"round {round_index} goal acceptance timeout")
            break
        handles = [future.result() for future in send_futures]
        if any(handle is None or not handle.accepted for handle in handles):
            failures.append(f"round {round_index} goal rejected")
            break
        result_futures = [handle.get_result_async() for handle in handles]
        started = time.monotonic()
        if not spin_until(node, result_futures, args.goal_timeout):
            for handle in handles:
                handle.cancel_goal_async()
            failures.append(f"round {round_index} timeout")
            break
        round_result = {"round": round_index, "robots": {}}
        for name, future in zip(names, result_futures):
            response = future.result()
            final = node.poses.get(name)
            target = targets[name]
            xy_error = math.inf if final is None else math.hypot(final[0] - target[0], final[1] - target[1])
            yaw_error = math.inf if final is None else angle_error(final[2], target[2])
            passed = response.status == GoalStatus.STATUS_SUCCEEDED and xy_error <= 0.25 and yaw_error <= 0.25
            if not passed:
                failures.append(
                    f"round {round_index} {name}: status={response.status} xy={xy_error:.3f} yaw={yaw_error:.3f}")
            round_result["robots"][name] = {
                "status": response.status, "xy_error_m": xy_error, "yaw_error_rad": yaw_error,
                "path_length_m": node.distance[name] - before[name], "passed": passed,
                "command_samples": node.command_samples[name] - commands_before[name],
                "zero_command_samples": node.zero_command_samples[name] - zero_before[name],
            }
        round_result["wall_duration_s"] = time.monotonic() - started
        results.append(round_result)
        if failures:
            break
    try:
        source_commit = subprocess.run(
            ["git", "-C", str(ROOT), "rev-parse", "HEAD"], check=True,
            capture_output=True, text=True, timeout=5.0,
        ).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        source_commit = "unknown"
    report = {
        "passed": not failures and len(results) == len(rounds),
        "source_commit": source_commit,
        "scenario": str(scenario_path),
        "scenario_sha256": hashlib.sha256(scenario_path.read_bytes()).hexdigest(),
        "tasks": str(tasks_path),
        "tasks_sha256": hashlib.sha256(tasks_path.read_bytes()).hexdigest(),
        "planner_algorithm": scenario.planner_algorithm,
        "rounds_requested": len(rounds), "rounds_completed": len(results),
        "minimum_center_separation_m": None if math.isinf(node.minimum_separation) else node.minimum_separation,
        "results": results, "failures": failures,
    }
    output = Path(args.output) if args.output else Path(
        os.environ.get("ARENA_MULTI_RUN_DIR", str(ROOT / "evidence/runs/current"))) / "navigation_benchmark.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
