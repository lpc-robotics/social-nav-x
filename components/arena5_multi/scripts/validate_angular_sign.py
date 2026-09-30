#!/usr/bin/env python3
"""Validate ROS angular.z sign and isolation on an external-control scenario."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/arena_multi_control"))
from arena_multi_control.scenario import load_scenario  # noqa: E402


def yaw_of(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_delta(current: float, previous: float) -> float:
    return (current - previous + math.pi) % (2.0 * math.pi) - math.pi


class Probe(Node):
    def __init__(self, names: list[str]) -> None:
        super().__init__("multirobot_angular_sign_probe")
        self.odom: dict[str, Odometry] = {}
        self.unwrapped = {name: 0.0 for name in names}
        self.last_yaw: dict[str, float] = {}
        self.angular_samples = {name: [] for name in names}
        self.publishers_by_name = {
            name: self.create_publisher(Twist, f"/{name}/cmd_vel_external", 10)
            for name in names
        }
        self._probe_subscriptions = [
            self.create_subscription(
                Odometry, f"/{name}/odom",
                lambda message, robot=name: self.on_odom(robot, message),
                qos_profile_sensor_data,
            )
            for name in names
        ]

    def on_odom(self, name: str, message: Odometry) -> None:
        yaw = yaw_of(message)
        if name in self.last_yaw:
            self.unwrapped[name] += angle_delta(yaw, self.last_yaw[name])
        self.last_yaw[name] = yaw
        self.odom[name] = message
        self.angular_samples[name].append(float(message.twist.twist.angular.z))

    def pump(self, commands: dict[str, float], duration: float) -> None:
        deadline = time.monotonic() + duration
        next_publish = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                for name, angular in commands.items():
                    command = Twist()
                    command.angular.z = angular
                    self.publishers_by_name[name].publish(command)
                next_publish = now + 0.05
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_ready(self, names: list[str], timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline and not all(name in self.odom for name in names):
            rclpy.spin_once(self, timeout_sec=0.05)
        return all(name in self.odom for name in names)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario")
    parser.add_argument("--phase-duration", type=float, default=8.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    scenario = load_scenario(args.scenario)
    names = [robot.name for robot in scenario.robots]
    if len(names) < 2 or any(robot.control_mode != "external" for robot in scenario.robots):
        raise SystemExit("angular sign validation requires at least two external-control robots")
    tested, idle = names[:2]

    rclpy.init()
    node = Probe(names)
    failures: list[str] = []
    measurements: dict[str, object] = {}
    try:
        if not node.wait_ready(names, 180.0):
            failures.append("odometry did not become ready")
        else:
            idle_start = node.odom[idle].pose.pose.position
            positive_start = node.unwrapped[tested]
            positive_sample_start = len(node.angular_samples[tested])
            node.pump({tested: 0.5, idle: 0.0}, args.phase_duration)
            positive_delta = node.unwrapped[tested] - positive_start
            positive_twist = node.angular_samples[tested][positive_sample_start:]
            node.pump({tested: 0.0, idle: 0.0}, 1.0)

            negative_start = node.unwrapped[tested]
            negative_sample_start = len(node.angular_samples[tested])
            node.pump({tested: -0.5, idle: 0.0}, args.phase_duration)
            negative_delta = node.unwrapped[tested] - negative_start
            negative_twist = node.angular_samples[tested][negative_sample_start:]
            node.pump({tested: 0.0, idle: 0.0}, 1.0)

            idle_end = node.odom[idle].pose.pose.position
            idle_drift = math.hypot(idle_end.x - idle_start.x, idle_end.y - idle_start.y)
            positive_mean = sum(positive_twist) / len(positive_twist) if positive_twist else 0.0
            negative_mean = sum(negative_twist) / len(negative_twist) if negative_twist else 0.0
            final_twist = float(node.odom[tested].twist.twist.angular.z)
            measurements = {
                "tested_robot": tested,
                "positive_command_rad_s": 0.5,
                "positive_unwrapped_yaw_delta_rad": positive_delta,
                "positive_mean_odom_angular_rad_s": positive_mean,
                "negative_command_rad_s": -0.5,
                "negative_unwrapped_yaw_delta_rad": negative_delta,
                "negative_mean_odom_angular_rad_s": negative_mean,
                "idle_robot": idle,
                "idle_translation_drift_m": idle_drift,
                "final_odom_angular_rad_s": final_twist,
            }
            if positive_delta <= 0.1 or positive_mean <= 0.05:
                failures.append("positive angular command did not produce positive yaw and odometry twist")
            if negative_delta >= -0.1 or negative_mean >= -0.05:
                failures.append("negative angular command did not produce negative yaw and odometry twist")
            if idle_drift > 0.02:
                failures.append(f"idle robot drift {idle_drift:.4f} m exceeds 0.02 m")
            if abs(final_twist) > 0.03:
                failures.append(f"final angular speed {final_twist:.4f} rad/s exceeds 0.03")
    finally:
        try:
            source_commit = subprocess.run(
                ["git", "-C", str(ROOT), "rev-parse", "HEAD"], check=True,
                capture_output=True, text=True, timeout=5.0,
            ).stdout.strip()
        except (OSError, subprocess.SubprocessError):
            source_commit = "unknown"
        report = {
            "passed": not failures,
            "source_commit": source_commit,
            "scenario": str(Path(args.scenario).resolve()),
            "phase_duration_wall_s": args.phase_duration,
            "measurements": measurements,
            "failures": failures,
        }
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
