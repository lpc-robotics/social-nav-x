#!/usr/bin/env python3
"""Exercise the standalone MPC visualization node over the ROS graph."""

import argparse
import json
import math
import time
from pathlib import Path as FilePath

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from hunav_msgs.msg import Agent, Agents
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import MarkerArray


def qos_profile(durability=DurabilityPolicy.VOLATILE):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=durability,
    )


class VisualizationProbe(Node):
    def __init__(self):
        super().__init__("mpc_visualization_smoke_probe")
        volatile = qos_profile()
        self.global_input = self.create_publisher(Path, "/plan", volatile)
        self.local_input = self.create_publisher(
            Path, "/FollowPath/predicted_path", volatile
        )
        self.human_input = self.create_publisher(Agents, "/human_states", volatile)
        self.global_output = None
        self.local_output = None
        self.human_output = None
        self.create_subscription(Path, "/mpc/global_plan", self.on_global, volatile)
        self.create_subscription(Path, "/mpc/local_trajectory", self.on_local, volatile)
        self.create_subscription(
            MarkerArray, "/mpc/human_markers", self.on_humans, volatile
        )

    def on_global(self, message):
        self.global_output = message

    def on_local(self, message):
        self.local_output = message

    def on_humans(self, message):
        self.human_output = message

    def publish_inputs(self):
        stamp = self.get_clock().now().to_msg()
        global_path = Path()
        global_path.header.frame_id = "map"
        global_path.header.stamp = stamp
        for x, y in ((1.0, 1.0), (2.0, 1.5), (3.0, 2.0)):
            pose = PoseStamped()
            # Nav2's installed Navfn planner leaves these per-pose frames
            # empty while setting Path.header.frame_id to map. The visualizer
            # must normalize this form for strict Foxglove Path validation.
            pose.header.stamp = stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            global_path.poses.append(pose)
        local_path = Path()
        local_path.header.frame_id = "map"
        local_path.header.stamp = stamp
        for x, y in ((1.0, 1.0), (1.2, 1.05), (1.4, 1.15)):
            pose = PoseStamped()
            pose.header = local_path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            local_path.poses.append(pose)
        humans = Agents()
        humans.header.frame_id = "map"
        humans.header.stamp = stamp
        agent = Agent()
        agent.id = 6
        agent.radius = 0.3
        agent.position.position.x = 2.0
        agent.position.position.y = 2.0
        agent.position.orientation.w = 1.0
        agent.velocity.linear.x = 0.4
        agent.velocity.linear.y = -0.2
        agent.behavior.type = 6
        goal = Pose()
        goal.position.x = 4.0
        goal.position.y = 3.0
        goal.orientation.w = 1.0
        agent.goals = [goal]
        humans.agents = [agent]
        self.global_input.publish(global_path)
        self.local_input.publish(local_path)
        self.human_input.publish(humans)


def endpoint_qos(node, topic):
    endpoints = node.get_publishers_info_by_topic(topic)
    return [
        {
            "node": endpoint.node_name,
            "reliability": endpoint.qos_profile.reliability.name,
            "durability": endpoint.qos_profile.durability.name,
            "depth": endpoint.qos_profile.depth,
        }
        for endpoint in endpoints
    ]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = VisualizationProbe()
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            node.publish_inputs()
            rclpy.spin_once(node, timeout_sec=0.1)
            if (
                node.global_output is not None
                and node.local_output is not None
                and node.human_output is not None
            ):
                break
        namespaces = sorted(
            {marker.ns for marker in (node.human_output.markers if node.human_output else [])}
        )
        safety_markers = [
            marker
            for marker in (node.human_output.markers if node.human_output else [])
            if marker.ns == "human_mpc_exclusion"
        ]
        expected_safety_diameter = 2.0 * (0.3 + 0.326 + 0.05 + 0.35)
        safety_diameter = safety_markers[0].scale.x if safety_markers else None
        expected_namespaces = {
            "human_body",
            "human_goals",
            "human_label",
            "human_mpc_exclusion",
            "human_prediction",
            "human_velocity",
        }
        qos = {
            topic: endpoint_qos(node, topic)
            for topic in (
                "/mpc/global_plan",
                "/mpc/local_trajectory",
                "/mpc/human_markers",
            )
        }
        passed = (
            node.global_output is not None
            and len(node.global_output.poses) == 3
            and all(
                pose.header.frame_id == node.global_output.header.frame_id
                for pose in node.global_output.poses
            )
            and node.local_output is not None
            and len(node.local_output.poses) == 3
            and node.human_output is not None
            and expected_namespaces.issubset(namespaces)
            and safety_diameter is not None
            and math.isclose(safety_diameter, expected_safety_diameter, abs_tol=1e-6)
            and all(qos.values())
        )
        report = {
            "gate": "PASS" if passed else "FAIL",
            "global_plan_header_frame": (
                node.global_output.header.frame_id if node.global_output else ""
            ),
            "global_plan_pose_count": (
                len(node.global_output.poses) if node.global_output else 0
            ),
            "global_plan_pose_frames": sorted(
                {
                    pose.header.frame_id
                    for pose in (node.global_output.poses if node.global_output else [])
                }
            ),
            "global_plan_source_fixture_pose_frames": ["<empty>"],
            "local_trajectory_pose_count": (
                len(node.local_output.poses) if node.local_output else 0
            ),
            "human_marker_count": (
                len(node.human_output.markers) if node.human_output else 0
            ),
            "human_marker_namespaces": namespaces,
            "safety_diameter_m": safety_diameter,
            "expected_safety_diameter_m": expected_safety_diameter,
            "publisher_qos": qos,
        }
        output = FilePath(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        print(json.dumps(report, indent=2, sort_keys=True))
        raise SystemExit(0 if passed else 2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
