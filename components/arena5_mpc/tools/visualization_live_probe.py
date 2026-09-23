#!/usr/bin/env python3
"""Observe visualization outputs without publishing into a live MPC graph."""

import argparse
import json
import math
import time
from pathlib import Path as FilePath

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import MarkerArray


class LiveVisualizationProbe(Node):
    def __init__(self):
        super().__init__("mpc_visualization_live_probe")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.global_paths = []
        self.local_paths = []
        self.human_markers = []
        self.create_subscription(Path, "/mpc/global_plan", self.global_paths.append, qos)
        self.create_subscription(
            Path, "/mpc/local_trajectory", self.local_paths.append, qos
        )
        self.create_subscription(
            MarkerArray, "/mpc/human_markers", self.human_markers.append, qos
        )


def finite_path(path):
    return (
        bool(path.header.frame_id)
        and bool(path.poses)
        and all(pose.header.frame_id == path.header.frame_id for pose in path.poses)
        and all(
            math.isfinite(value)
            for pose in path.poses
            for value in (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            )
        )
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--timeout", type=float, default=180.0)
    args = parser.parse_args()
    rclpy.init()
    node = LiveVisualizationProbe()
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            maximum_local_pose_count = max(
                (len(path.poses) for path in node.local_paths), default=0
            )
            if (
                node.global_paths
                and len(node.local_paths) >= 5
                and maximum_local_pose_count >= 2
                and len(node.human_markers) >= 5
            ):
                break
        namespaces = sorted(
            {
                marker.ns
                for message in node.human_markers
                for marker in message.markers
                if marker.ns
            }
        )
        expected_namespaces = {
            "human_body",
            "human_goals",
            "human_label",
            "human_mpc_exclusion",
            "human_prediction",
            "human_velocity",
        }
        maximum_local_pose_count = max(
            (len(path.poses) for path in node.local_paths), default=0
        )
        publishers = {
            topic: sorted(
                {
                    endpoint.node_name
                    for endpoint in node.get_publishers_info_by_topic(topic)
                }
            )
            for topic in (
                "/mpc/global_plan",
                "/mpc/local_trajectory",
                "/mpc/human_markers",
            )
        }
        passed = (
            bool(node.global_paths)
            and finite_path(node.global_paths[-1])
            and len(node.local_paths) >= 5
            and all(finite_path(path) for path in node.local_paths)
            and maximum_local_pose_count >= 2
            and len(node.human_markers) >= 5
            and expected_namespaces.issubset(namespaces)
            and all("mpc_visualizer" in names for names in publishers.values())
        )
        report = {
            "gate": "PASS" if passed else "FAIL",
            "global_plan_messages": len(node.global_paths),
            "global_plan_pose_count": len(node.global_paths[-1].poses) if node.global_paths else 0,
            "local_trajectory_messages": len(node.local_paths),
            "local_trajectory_last_pose_count": (
                len(node.local_paths[-1].poses) if node.local_paths else 0
            ),
            "local_trajectory_maximum_pose_count": maximum_local_pose_count,
            "human_marker_messages": len(node.human_markers),
            "human_marker_namespaces": namespaces,
            "publishers": publishers,
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
