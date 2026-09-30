#!/usr/bin/env python3
"""Validate the single-world, robot_1-referenced HuNav integration."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import re
import sys
import time

from hunav_msgs.msg import Agents
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/arena_multi_control"))
from arena_multi_control.scenario import load_scenario  # noqa: E402


class Probe(Node):
    def __init__(self, robot_names: list[str]) -> None:
        super().__init__("multirobot_hunav_runtime_probe")
        self.status_lines: list[str] = []
        self.agent_samples = []
        self.scan_counts = {name: 0 for name in robot_names}
        self.owned_subscriptions = [
            self.create_subscription(
                String, "/multirobot/hunav/status", self._on_status, 20
            ),
            self.create_subscription(
                Agents, "/multirobot/hunav/agents", self._on_agents, 20
            ),
        ]
        self.owned_subscriptions.extend(
            self.create_subscription(
                LaserScan,
                f"/{name}/lidar_normalized",
                lambda _message, robot=name: self._on_scan(robot),
                qos_profile_sensor_data,
            )
            for name in robot_names
        )

    def _on_status(self, message: String) -> None:
        self.status_lines.append(message.data)

    def _on_agents(self, message: Agents) -> None:
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        agents = {
            agent.name: {
                "id": int(agent.id),
                "type": int(agent.behavior.type),
                "state": int(agent.behavior.state),
                "x": float(agent.position.position.x),
                "y": float(agent.position.position.y),
            }
            for agent in message.agents
        }
        self.agent_samples.append((stamp, message.header.frame_id, agents))

    def _on_scan(self, robot: str) -> None:
        self.scan_counts[robot] += 1


def expected_agents(profile: str) -> tuple[Path, list[str], list[int]]:
    filename = "regular.yaml" if profile == "regular" else "six_behaviors.yaml"
    path = ROOT / "config/hunav" / filename
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    params = document["hunav_loader"]["ros__parameters"]
    names = list(params["agents"])
    types = sorted(int(params[name]["behavior"]["type"]) for name in names)
    return path, names, types


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    scenario = load_scenario(args.scenario)
    if scenario.hunav_profile not in ("regular", "six"):
        raise SystemExit("scenario must select the regular or six HuNav profile")
    robot_names = [robot.name for robot in scenario.robots]
    config_path, expected_names, expected_types = expected_agents(scenario.hunav_profile)

    rclpy.init()
    node = Probe(robot_names)
    deadline = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    failures = []
    graph_nodes = [
        f"{namespace.rstrip('/')}/{name}" if namespace != "/" else f"/{name}"
        for name, namespace in node.get_node_names_and_namespaces()
    ]
    expected_graph_nodes = [
        "/hunav_loader", "/hunav_agent_manager", "/arena_hunav_isaac_bridge"
    ]
    node_counts = {name: graph_nodes.count(name) for name in expected_graph_nodes}
    for name, count in node_counts.items():
        if count != 1:
            failures.append(f"{name} node count={count}")

    agent_publishers = len(node.get_publishers_info_by_topic("/multirobot/hunav/agents"))
    if agent_publishers != 1:
        failures.append(f"agent state publishers={agent_publishers}")
    if len(node.agent_samples) < 2:
        failures.append("insufficient HuNav agent state samples")

    observed_names = []
    observed_types = []
    frame_ids = []
    stamp_span = 0.0
    maximum_displacement = 0.0
    if node.agent_samples:
        first_stamp, _, first = node.agent_samples[0]
        last_stamp, _, last = node.agent_samples[-1]
        stamp_span = last_stamp - first_stamp
        observed_names = sorted(last)
        observed_types = sorted(item["type"] for item in last.values())
        frame_ids = sorted({sample[1] for sample in node.agent_samples})
        for name in set(first).intersection(last):
            maximum_displacement = max(
                maximum_displacement,
                math.hypot(
                    last[name]["x"] - first[name]["x"],
                    last[name]["y"] - first[name]["y"],
                ),
            )
        if stamp_span <= 0.0:
            failures.append(f"HuNav timestamps did not advance: span={stamp_span:.6f}")
        if frame_ids != ["map"]:
            failures.append(f"HuNav frame ids={frame_ids}")
    if observed_names != sorted(expected_names):
        failures.append(f"agent names={observed_names}")
    if observed_types != expected_types:
        failures.append(f"behavior types={observed_types}")

    running = [line for line in node.status_lines if line.startswith("SIX_BEHAVIORS_RUNNING")]
    scoped = [
        line for line in node.status_lines
        if "reference=robot_1" in line
        and "interaction_scope=single_reference_robot" in line
    ]
    if not running:
        failures.append("periodic HuNav runtime status missing")
    if not scoped:
        failures.append("robot_1 reference scope status missing")
    max_dt_values = []
    compute_counts = []
    update_counts = []
    for line in running:
        for pattern, target in (
            (r"\bmax_dt=([0-9.]+)", max_dt_values),
            (r"\bcompute=(\d+)", compute_counts),
            (r"\bupdates=(\d+)", update_counts),
        ):
            match = re.search(pattern, line)
            if match:
                target.append(float(match.group(1)))
    if max_dt_values and max(max_dt_values) > 0.0255:
        failures.append(f"maximum HuNav integration step={max(max_dt_values):.6f}")
    if len(compute_counts) >= 2 and compute_counts[-1] <= compute_counts[0]:
        failures.append("HuNav compute counter did not advance")
    if len(update_counts) >= 2 and update_counts[-1] <= update_counts[0]:
        failures.append("Isaac pedestrian update counter did not advance")
    for name, count in node.scan_counts.items():
        if count < 2:
            failures.append(f"{name} normalized lidar unavailable")

    report = {
        "passed": not failures,
        "profile": scenario.hunav_profile,
        "config": str(config_path),
        "duration_wall_s": args.duration,
        "node_counts": node_counts,
        "agent_state_publishers": agent_publishers,
        "agent_samples": len(node.agent_samples),
        "agent_stamp_span_sim_s": stamp_span,
        "agent_maximum_displacement_m": maximum_displacement,
        "agent_names": observed_names,
        "behavior_types": observed_types,
        "status_samples": len(node.status_lines),
        "runtime_status_samples": len(running),
        "reference_scope_status_samples": len(scoped),
        "max_integration_step_s": max(max_dt_values) if max_dt_values else None,
        "compute_count_range": [min(compute_counts), max(compute_counts)] if compute_counts else [],
        "update_count_range": [min(update_counts), max(update_counts)] if update_counts else [],
        "normalized_lidar_messages": node.scan_counts,
        "failures": failures,
    }
    output = Path(args.output) if args.output else Path(
        os.environ.get("ARENA_MULTI_RUN_DIR", str(ROOT / "evidence/runs/current"))
    ) / "hunav_runtime_validation.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
