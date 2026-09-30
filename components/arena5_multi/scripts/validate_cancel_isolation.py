#!/usr/bin/env python3
"""Cancel one robot's Nav2 goal while the other robot continues."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/arena_multi_control"))
from arena_multi_control.scenario import load_scenario  # noqa: E402


def spin_until(node, futures, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline and not all(item.done() for item in futures):
        rclpy.spin_once(node, timeout_sec=0.05)
    return all(item.done() for item in futures)


class Probe(Node):
    def __init__(self, names: list[str]) -> None:
        super().__init__("multirobot_cancel_isolation_probe")
        self.poses = {}
        self.action_clients = {
            name: ActionClient(self, NavigateToPose, f"/{name}/navigate_to_pose")
            for name in names
        }
        self._probe_subscriptions = [
            self.create_subscription(
                Odometry, f"/{name}/odom",
                lambda message, robot=name: self.poses.__setitem__(
                    robot, (message.pose.pose.position.x, message.pose.pose.position.y)
                ), qos_profile_sensor_data,
            ) for name in names
        ]

    def goal(self, x: float, y: float) -> NavigateToPose.Goal:
        result = NavigateToPose.Goal()
        result.pose = PoseStamped()
        result.pose.header.frame_id = "map"
        result.pose.header.stamp = self.get_clock().now().to_msg()
        result.pose.pose.position.x = x
        result.pose.pose.position.y = y
        result.pose.pose.orientation.w = 1.0
        return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario")
    parser.add_argument("--cancel-after", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=600.0)
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    scenario = load_scenario(args.scenario)
    robots = [robot for robot in scenario.robots if robot.control_mode == "nav2"]
    if len(robots) != 2:
        raise SystemExit("cancel isolation requires exactly two Nav2 robots")
    names = [robot.name for robot in robots]
    first, second = names
    rclpy.init()
    node = Probe(names)
    failures = []
    statuses = {}
    targets = {}
    second_motion = None
    try:
        pose_deadline = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < pose_deadline and any(name not in node.poses for name in names):
            rclpy.spin_once(node, timeout_sec=0.05)
        if any(name not in node.poses for name in names):
            failures.append("initial odometry unavailable")
        else:
            for name, distance in ((first, 4.0), (second, 3.0)):
                x, y = node.poses[name]
                target_x = x + distance if x + distance <= 28.0 else x - distance
                targets[name] = (target_x, y)

        if failures:
            pass
        elif any(not client.wait_for_server(timeout_sec=30.0) for client in node.action_clients.values()):
            failures.append("NavigateToPose action server unavailable")
        else:
            send = [node.action_clients[name].send_goal_async(node.goal(*targets[name])) for name in names]
            if not spin_until(node, send, 30.0):
                failures.append("goal acceptance timeout")
            else:
                handles = {name: future.result() for name, future in zip(names, send)}
                if any(handle is None or not handle.accepted for handle in handles.values()):
                    failures.append("goal rejected")
                else:
                    second_before = node.poses.get(second)
                    deadline = time.monotonic() + args.cancel_after
                    while rclpy.ok() and time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.05)
                    cancel = handles[first].cancel_goal_async()
                    if not spin_until(node, [cancel], 10.0) or not cancel.result().goals_canceling:
                        failures.append(f"{first} cancellation was not accepted")
                    results = {name: handle.get_result_async() for name, handle in handles.items()}
                    if not spin_until(node, list(results.values()), args.timeout):
                        failures.append("action result timeout")
                    else:
                        statuses = {name: future.result().status for name, future in results.items()}
                        if statuses[first] != GoalStatus.STATUS_CANCELED:
                            failures.append(f"{first} status after cancel={statuses[first]}")
                        if statuses[second] != GoalStatus.STATUS_SUCCEEDED:
                            failures.append(f"{second} status after peer cancel={statuses[second]}")
                        second_after = node.poses.get(second)
                        second_motion = (
                            0.0 if second_before is None or second_after is None
                            else math.hypot(second_after[0] - second_before[0], second_after[1] - second_before[1])
                        )
                        if second_motion < 0.25:
                            failures.append(f"{second} did not continue after peer cancellation")
    finally:
        report = {"passed": not failures, "statuses": statuses, "targets": targets,
                  "uncanceled_robot_motion_m": second_motion,
                  "failures": failures}
        output = Path(args.output) if args.output else Path(
            os.environ.get("ARENA_MULTI_RUN_DIR", str(ROOT / "evidence/runs/current"))
        ) / "cancel_isolation.json"
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
