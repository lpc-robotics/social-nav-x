#!/usr/bin/env python3
import argparse
import json
import time
from collections import defaultdict
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class GraphProbe(Node):
    def __init__(self):
        super().__init__("arena_mpc_p2_graph_probe")
        self.parents = defaultdict(set)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(TFMessage, "/tf", self.on_tf, qos)

    def on_tf(self, message):
        for transform in message.transforms:
            self.parents[transform.child_frame_id].add(transform.header.frame_id)

    def publishers(self, topic):
        return [
            {
                "node_name": endpoint.node_name,
                "node_namespace": endpoint.node_namespace,
                "topic_type": endpoint.topic_type,
                "reliability": str(endpoint.qos_profile.reliability),
                "durability": str(endpoint.qos_profile.durability),
            }
            for endpoint in self.get_publishers_info_by_topic(topic)
        ]

    def report(self):
        topics = {
            topic: self.publishers(topic)
            for topic in ("/cmd_vel", "/odom", "/tf", "/tf_static")
        }
        multiple_parents = {
            child: sorted(parents)
            for child, parents in self.parents.items()
            if len(parents) > 1
        }
        return {
            "publishers": topics,
            "tf_parent_by_child": {
                child: sorted(parents) for child, parents in sorted(self.parents.items())
            },
            "tf_children_with_multiple_parents": multiple_parents,
            "pass": len(topics["/cmd_vel"]) == 1
            and topics["/cmd_vel"][0]["node_name"] == "mpc_command_watchdog"
            and len(topics["/odom"]) == 1
            and not multiple_parents,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--output")
    args = parser.parse_args()
    rclpy.init()
    probe = GraphProbe()
    deadline = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.05)
    report = probe.report()
    rendered = json.dumps(report, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered + "\n", encoding="utf-8")
    probe.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if report["pass"] else 2)


if __name__ == "__main__":
    main()
