#!/usr/bin/env python3
"""Measure the live multi-robot graph without sending motion commands."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import Buffer, TransformListener


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/arena_multi_control"))
from arena_multi_control.scenario import load_scenario  # noqa: E402


class Probe(Node):
    def __init__(self, names):
        super().__init__("multirobot_runtime_probe")
        self.started_monotonic = time.monotonic()
        self.clock_count = 0
        self.clock_last = None
        self.clock_samples = []
        self.odom = {name: [] for name in names}
        self.scans = {name: [] for name in names}
        self.joints = {name: [] for name in names}
        self.commands = {name: [] for name in names}
        self.diagnostics = {name: [] for name in names}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_clients = {
            name: ActionClient(self, NavigateToPose, f"/{name}/navigate_to_pose")
            for name in names
        }
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
        self.create_subscription(
            DiagnosticArray, "/multirobot/diagnostics", self.on_diagnostic, 20
        )
        self._probe_subscriptions = []
        for name in names:
            self._probe_subscriptions.append(self.create_subscription(
                Odometry, f"/{name}/odom", lambda msg, n=name: self.on_odom(n, msg), qos_profile_sensor_data))
            self._probe_subscriptions.append(self.create_subscription(
                LaserScan, f"/{name}/lidar_normalized", lambda msg, n=name: self.on_scan(n, msg), qos_profile_sensor_data))
            self._probe_subscriptions.append(self.create_subscription(
                JointState, f"/{name}/joint_states",
                lambda msg, n=name: self.joints[n].append(time.monotonic()), qos_profile_sensor_data))
            self._probe_subscriptions.append(self.create_subscription(
                Twist, f"/{name}/cmd_vel",
                lambda msg, n=name: self.commands[n].append(time.monotonic()), 10))

    def on_clock(self, message):
        stamp = message.clock.sec + message.clock.nanosec * 1e-9
        if self.clock_last is not None and stamp < self.clock_last:
            raise RuntimeError("simulation clock moved backwards")
        self.clock_last = stamp
        self.clock_count += 1
        self.clock_samples.append((stamp, time.monotonic()))

    def on_diagnostic(self, message):
        now = time.monotonic()
        for status in message.status:
            prefix = "multirobot/"
            suffix = "/command_guard"
            if status.name.startswith(prefix) and status.name.endswith(suffix):
                name = status.name[len(prefix):-len(suffix)]
                if name in self.diagnostics:
                    values = {item.key: item.value for item in status.values}
                    self.diagnostics[name].append((now, status.message, values))

    def on_odom(self, name, message):
        self.odom[name].append((message.pose.pose.position.x, message.pose.pose.position.y,
                                time.monotonic(), message.header.frame_id, message.child_frame_id))

    def on_scan(self, name, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        sim_age = None if self.clock_last is None else self.clock_last - stamp
        self.scans[name].append((stamp, message.header.frame_id, time.monotonic(), sim_age))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    scenario = load_scenario(args.scenario)
    names = [robot.name for robot in scenario.robots]
    rclpy.init()
    node = Probe(names)
    deadline = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    failures = []
    publishers = {"clock": len(node.get_publishers_info_by_topic("/clock"))}
    if publishers["clock"] != 1:
        failures.append(f"/clock publishers={publishers['clock']}")
    robots = {}
    for name in names:
        cmd_publishers = len(node.get_publishers_info_by_topic(f"/{name}/cmd_vel"))
        publishers[name + "/cmd_vel"] = cmd_publishers
        if cmd_publishers != 1:
            failures.append(f"/{name}/cmd_vel publishers={cmd_publishers}")
        odom = node.odom[name]
        scans = node.scans[name]
        matching_robot = next(robot for robot in scenario.robots if robot.name == name)
        if len(odom) < 2:
            failures.append(f"{name} odom missing")
        if len(scans) < 2:
            failures.append(f"{name} normalized scan missing")
        if len(node.joints[name]) < 2:
            failures.append(f"{name} joint states missing")
        sim_hz = 0.0
        wall_hz = 0.0
        if len(scans) >= 2:
            sim_span = scans[-1][0] - scans[0][0]
            wall_span = scans[-1][2] - scans[0][2]
            sim_hz = (len(scans) - 1) / sim_span if sim_span > 0 else 0.0
            wall_hz = (len(scans) - 1) / wall_span if wall_span > 0 else 0.0
            if not 9.5 <= sim_hz <= 10.5:
                failures.append(f"{name} scan simulation frequency={sim_hz:.3f}")
            expected_frame = f"{name}/lidar_link_normalized"
            if any(frame != expected_frame for _, frame, _, _ in scans):
                failures.append(f"{name} scan frame mismatch")
        drift = 0.0
        if len(odom) >= 2:
            drift = math.hypot(odom[-1][0] - odom[0][0], odom[-1][1] - odom[0][1])
            if drift > 0.02:
                failures.append(f"{name} idle drift={drift:.4f}")
            spawn_error = math.hypot(
                odom[0][0] - matching_robot.x,
                odom[0][1] - matching_robot.y,
            )
            if spawn_error > 0.05:
                failures.append(f"{name} initial spawn error={spawn_error:.4f}")
            if any(item[3] != f"{name}/odom" or item[4] != f"{name}/base_link" for item in odom):
                failures.append(f"{name} odometry frame mismatch")
        else:
            spawn_error = None
        diagnostic_samples = node.diagnostics[name]
        settled_diagnostics = [
            item for item in diagnostic_samples
            if item[0] - node.started_monotonic >= 5.0
        ]
        accepted_idle_reasons = {"ok", "command_stale"}
        observation_faults = [
            item[1] for item in settled_diagnostics
            if item[1] not in accepted_idle_reasons
        ]
        if len(settled_diagnostics) < 2:
            failures.append(f"{name} command guard diagnostics missing")
        if observation_faults:
            failures.append(
                f"{name} command guard observation faults={sorted(set(observation_faults))}"
            )
        maximum_lidar_wall_age = max(
            (float(item[2]["lidar_wall_age_s"]) for item in settled_diagnostics
             if "lidar_wall_age_s" in item[2]),
            default=None,
        )
        maximum_lidar_sim_age = max(
            (float(item[2]["lidar_sim_age_s"]) for item in settled_diagnostics
             if "lidar_sim_age_s" in item[2]),
            default=None,
        )
        command_hz = 0.0
        if len(node.commands[name]) >= 2:
            command_span = node.commands[name][-1] - node.commands[name][0]
            command_hz = (len(node.commands[name]) - 1) / command_span if command_span > 0 else 0.0
            if not 18.0 <= command_hz <= 22.0:
                failures.append(f"{name} guard output wall frequency={command_hz:.3f}")
        else:
            failures.append(f"{name} final command output missing")
        try:
            node.tf_buffer.lookup_transform("map", f"{name}/base_link", Time())
            tf_ready = True
        except Exception as error:
            tf_ready = False
            failures.append(f"{name} TF unavailable: {error}")
        action_ready = True
        if matching_robot.control_mode == "nav2":
            action_ready = node.action_clients[name].wait_for_server(timeout_sec=0.0)
            if not action_ready:
                failures.append(f"{name} NavigateToPose action unavailable")
        sim_ages = [item[3] for item in scans if item[3] is not None]
        robots[name] = {
            "odom_messages": len(odom), "scan_messages": len(scans),
            "joint_state_messages": len(node.joints[name]),
            "scan_sim_hz": sim_hz, "scan_wall_hz": wall_hz,
            "scan_sim_age_mean_s": sum(sim_ages) / len(sim_ages) if sim_ages else None,
            "scan_sim_age_max_s": max(sim_ages) if sim_ages else None,
            "guard_output_wall_hz": command_hz,
            "guard_diagnostic_samples": len(settled_diagnostics),
            "guard_observation_faults": sorted(set(observation_faults)),
            "maximum_lidar_wall_age_s": maximum_lidar_wall_age,
            "maximum_lidar_sim_age_s": maximum_lidar_sim_age,
            "idle_drift_m": drift, "initial_spawn_error_m": spawn_error,
            "tf_map_to_base_ready": tf_ready, "navigate_to_pose_ready": action_ready,
        }
    rtf = 0.0
    if len(node.clock_samples) >= 2:
        sim_span = node.clock_samples[-1][0] - node.clock_samples[0][0]
        wall_span = node.clock_samples[-1][1] - node.clock_samples[0][1]
        rtf = sim_span / wall_span if wall_span > 0 else 0.0
    report = {"passed": not failures, "duration_wall_s": args.duration, "publishers": publishers,
              "real_time_factor": rtf, "robots": robots, "failures": failures}
    output = Path(args.output) if args.output else Path(
        os.environ.get("ARENA_MULTI_RUN_DIR", str(ROOT / "evidence/runs/current"))) / "runtime_validation.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
