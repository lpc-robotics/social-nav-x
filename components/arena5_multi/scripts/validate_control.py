#!/usr/bin/env python3
"""Exercise two external-control robots and save P1 isolation evidence."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/arena_multi_control"))
from arena_multi_control.scenario import load_scenario  # noqa: E402


def planar_speed(message: Odometry) -> float:
    velocity = message.twist.twist
    return math.hypot(velocity.linear.x, velocity.linear.y)


class ControlProbe(Node):
    def __init__(self, names: list[str]) -> None:
        super().__init__("multirobot_control_probe")
        self.names = names
        self.odom: dict[str, Odometry] = {}
        self.scan_seen: dict[str, float] = {}
        self.final_commands: dict[str, list[tuple[float, float, float]]] = {
            name: [] for name in names
        }
        self.command_publishers = {
            name: self.create_publisher(Twist, f"/{name}/cmd_vel_external", 10)
            for name in names
        }
        self.estops = {
            name: self.create_client(SetBool, f"/{name}/emergency_stop") for name in names
        }
        self._probe_subscriptions = []
        for name in names:
            self._probe_subscriptions.extend([
                self.create_subscription(
                    Odometry, f"/{name}/odom",
                    lambda message, robot=name: self.odom.__setitem__(robot, message),
                    qos_profile_sensor_data,
                ),
                self.create_subscription(
                    LaserScan, f"/{name}/lidar_normalized",
                    lambda _message, robot=name: self.scan_seen.__setitem__(robot, time.monotonic()),
                    qos_profile_sensor_data,
                ),
                self.create_subscription(
                    Twist, f"/{name}/cmd_vel",
                    lambda message, robot=name: self._on_final(robot, message), 10,
                ),
            ])

    def _on_final(self, name: str, message: Twist) -> None:
        self.final_commands[name].append(
            (time.monotonic(), float(message.linear.x), float(message.angular.z))
        )

    def pose(self, name: str) -> tuple[float, float]:
        position = self.odom[name].pose.pose.position
        return float(position.x), float(position.y)

    def displacement(self, name: str, start: tuple[float, float]) -> float:
        current = self.pose(name)
        return math.hypot(current[0] - start[0], current[1] - start[1])

    @staticmethod
    def command(linear: float = 0.0, angular: float = 0.0) -> Twist:
        result = Twist()
        result.linear.x = linear
        result.angular.z = angular
        return result

    def pump(self, commands: dict[str, Twist], timeout: float, predicate=None) -> bool:
        deadline = time.monotonic() + timeout
        next_publish = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                for name, command in commands.items():
                    self.command_publishers[name].publish(command)
                next_publish = now + 0.05
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate is not None and predicate():
                return True
        return predicate is None

    def wait_ready(self, timeout: float) -> bool:
        return self.pump({}, timeout, lambda: all(
            name in self.odom and name in self.scan_seen for name in self.names
        ))

    def wait_final(self, name: str, predicate, timeout: float) -> float | None:
        started = time.monotonic()
        seen = len(self.final_commands[name])
        deadline = started + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            for sample in self.final_commands[name][seen:]:
                if predicate(sample[1], sample[2]):
                    return sample[0] - started
            seen = len(self.final_commands[name])
        return None

    def set_estop(self, name: str, enabled: bool, timeout: float = 5.0):
        client = self.estops[name]
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        request = SetBool.Request()
        request.data = enabled
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.02)
        return future.result() if future.done() else None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario")
    parser.add_argument("--output", default="")
    parser.add_argument("--motion-timeout", type=float, default=60.0)
    args = parser.parse_args()
    scenario = load_scenario(args.scenario)
    names = [robot.name for robot in scenario.robots]
    if len(names) != 2 or any(robot.control_mode != "external" for robot in scenario.robots):
        raise SystemExit("control validation requires exactly two external-mode robots")
    first, second = names

    rclpy.init()
    node = ControlProbe(names)
    failures: list[str] = []
    measurements: dict[str, object] = {}
    try:
        if not node.wait_ready(180.0):
            failures.append("odometry or normalized scan did not become ready")
            raise RuntimeError(failures[-1])

        configured = {robot.name: (robot.x, robot.y) for robot in scenario.robots}
        spawn_errors = {
            name: math.hypot(
                node.pose(name)[0] - configured[name][0],
                node.pose(name)[1] - configured[name][1],
            )
            for name in names
        }
        measurements["initial_spawn_error_m"] = spawn_errors
        bad_spawns = {name: error for name, error in spawn_errors.items() if error > 0.05}
        if bad_spawns:
            failures.append(f"initial spawn error exceeds 0.05 m: {bad_spawns}")
            raise RuntimeError(failures[-1])

        start_first = node.pose(first)
        start_second = node.pose(second)
        moved = node.pump(
            {first: node.command(0.20), second: node.command()},
            args.motion_timeout,
            lambda: node.displacement(first, start_first) >= 0.10,
        )
        first_motion = node.displacement(first, start_first)
        second_idle_drift = node.displacement(second, start_second)
        measurements["single_robot_motion"] = {
            first: first_motion, f"{second}_idle_drift_m": second_idle_drift,
        }
        if not moved or first_motion < 0.10:
            failures.append(f"{first} did not move 0.10 m")
        if second_idle_drift > 0.02:
            failures.append(f"{second} idle drift {second_idle_drift:.4f} m exceeds 0.02 m")

        concurrent_start = {name: node.pose(name) for name in names}
        both_moved = node.pump(
            {first: node.command(0.12, 0.0), second: node.command(0.10, 0.35)},
            args.motion_timeout,
            lambda: all(node.displacement(name, concurrent_start[name]) >= 0.05 for name in names),
        )
        concurrent_distance = {
            name: node.displacement(name, concurrent_start[name]) for name in names
        }
        measurements["simultaneous_different_commands_m"] = concurrent_distance
        if not both_moved:
            failures.append("robots did not independently follow simultaneous different commands")

        second_continue_start = node.pose(second)
        node.pump(
            {first: node.command(), second: node.command(0.10, 0.35)}, 2.0
        )
        continued = node.displacement(second, second_continue_start)
        measurements["stop_one_second_continues_m"] = continued
        if continued <= 0.005:
            failures.append(f"{second} did not continue after {first} was stopped")

        node.command_publishers[second].publish(node.command(0.10, 0.35))
        timeout_started = time.monotonic()
        zero_delay = node.wait_final(
            second, lambda linear, angular: abs(linear) < 1e-6 and abs(angular) < 1e-6, 1.0
        )
        command_stop_delay = None if zero_delay is None else time.monotonic() - timeout_started
        measurements["command_timeout_zero_output_wall_s"] = command_stop_delay
        if command_stop_delay is None or command_stop_delay > 0.6:
            failures.append(f"{second} command timeout exceeded 0.6 s: {command_stop_delay}")

        physical_started = time.monotonic()
        physical_stop_delay = None
        while rclpy.ok() and time.monotonic() - physical_started < 10.0:
            rclpy.spin_once(node, timeout_sec=0.02)
            message = node.odom.get(second)
            if message and planar_speed(message) <= 0.01 and abs(message.twist.twist.angular.z) <= 0.03:
                physical_stop_delay = time.monotonic() - physical_started
                break
        measurements["d6_physical_stop_after_zero_wall_s"] = physical_stop_delay
        if physical_stop_delay is None:
            failures.append(f"{second} physical velocity did not settle within 10 s")

        node.pump({first: node.command(0.10)}, 1.0)
        enabled = node.set_estop(first, True)
        estop_delay = node.wait_final(
            first, lambda linear, angular: abs(linear) < 1e-6 and abs(angular) < 1e-6, 0.3
        )
        cleared = node.set_estop(first, False)
        node.pump({}, 0.2)
        latest = node.final_commands[first][-1] if node.final_commands[first] else None
        stayed_zero = bool(latest and abs(latest[1]) < 1e-6 and abs(latest[2]) < 1e-6)
        measurements["emergency_stop"] = {
            "enable_success": bool(enabled and enabled.success),
            "zero_output_wall_s": estop_delay,
            "clear_success": bool(cleared and cleared.success),
            "requires_new_command": stayed_zero,
        }
        if not enabled or not enabled.success or estop_delay is None:
            failures.append(f"{first} emergency stop failed")
        if not cleared or not cleared.success or not stayed_zero:
            failures.append(f"{first} emergency clear did not require a new command")
    except RuntimeError:
        pass
    finally:
        report = {"passed": not failures, "scenario": str(Path(args.scenario).resolve()),
                  "measurements": measurements, "failures": failures}
        output = Path(args.output) if args.output else Path(
            os.environ.get("ARENA_MULTI_RUN_DIR", str(ROOT / "evidence/runs/current"))
        ) / "control_validation.json"
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
