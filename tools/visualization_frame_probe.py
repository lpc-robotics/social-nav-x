#!/usr/bin/env python3
"""Inspect Path pose frames and TF messages in a running MPC graph."""

import argparse
import json
import time
from collections import Counter
from pathlib import Path as FilePath

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


PATH_TOPICS = (
    "/plan",
    "/mpc/global_plan",
    "/FollowPath/predicted_path",
    "/mpc/local_trajectory",
)


def volatile_qos(depth=10):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class FrameProbe(Node):
    def __init__(self):
        super().__init__("mpc_visualization_frame_probe")
        self.paths = {}
        self.tf_messages = {"/tf": [], "/tf_static": []}
        for topic in PATH_TOPICS:
            self.create_subscription(
                Path,
                topic,
                lambda message, name=topic: self.paths.__setitem__(name, message),
                volatile_qos(),
            )
        self.create_subscription(
            TFMessage,
            "/tf",
            lambda message: self.tf_messages["/tf"].append(message),
            volatile_qos(100),
        )
        static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            TFMessage,
            "/tf_static",
            lambda message: self.tf_messages["/tf_static"].append(message),
            static_qos,
        )


def path_report(message):
    pose_frames = Counter(pose.header.frame_id or "<empty>" for pose in message.poses)
    return {
        "header_frame": message.header.frame_id or "<empty>",
        "pose_count": len(message.poses),
        "pose_frames": dict(sorted(pose_frames.items())),
        "all_pose_frames_match_path": bool(message.header.frame_id)
        and bool(message.poses)
        and all(
            pose.header.frame_id == message.header.frame_id for pose in message.poses
        ),
    }


def tf_report(messages):
    transforms = [transform for message in messages for transform in message.transforms]
    empty = [
        {
            "parent": transform.header.frame_id,
            "child": transform.child_frame_id,
        }
        for transform in transforms
        if not transform.header.frame_id or not transform.child_frame_id
    ]
    frames = sorted(
        {
            frame
            for transform in transforms
            for frame in (transform.header.frame_id, transform.child_frame_id)
            if frame
        }
    )
    return {
        "message_count": len(messages),
        "transform_count": len(transforms),
        "empty_frame_transforms": empty,
        "frames": frames,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--timeout", type=float, default=180.0)
    args = parser.parse_args()

    rclpy.init()
    node = FrameProbe()
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if len(node.paths) == len(PATH_TOPICS) and all(node.tf_messages.values()):
                break
        paths = {
            topic: path_report(message) for topic, message in sorted(node.paths.items())
        }
        tf = {
            topic: tf_report(messages)
            for topic, messages in sorted(node.tf_messages.items())
        }
        outputs_consistent = all(
            paths.get(topic, {}).get("all_pose_frames_match_path", False)
            for topic in ("/mpc/global_plan", "/mpc/local_trajectory")
        )
        tf_has_empty_frame = any(
            report["empty_frame_transforms"] for report in tf.values()
        )
        passed = outputs_consistent and not tf_has_empty_frame
        report = {
            "gate": "PASS" if passed else "FAIL",
            "paths": paths,
            "tf": tf,
            "tf_has_empty_frame": tf_has_empty_frame,
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
