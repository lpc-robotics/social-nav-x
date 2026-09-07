#!/usr/bin/env python3
"""Observe the unmodified P0 baseline without publishing ROS data."""

import argparse
import json
import math
import statistics
import time

import rclpy
from hunav_msgs.msg import Agents
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan


TOPICS = {
    "/human_states": Agents,
    "/odom": Odometry,
    "/lidar": LaserScan,
    "/clock": Clock,
    "/local_costmap/costmap": OccupancyGrid,
    "/global_costmap/costmap": OccupancyGrid,
}


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quantile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


class Observer(Node):
    def __init__(self):
        super().__init__(
            "arena5_p0_observer",
            enable_rosout=False,
            start_parameter_services=False,
        )
        self.clock_stamp = None
        self.records = {
            topic: {
                "receive": [],
                "stamp": [],
                "age": [],
                "frame_ids": set(),
                "agent_ids": set(),
                "agent_counts": set(),
            }
            for topic in TOPICS
        }
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._topic_subscriptions = []
        for topic, message_type in TOPICS.items():
            self._topic_subscriptions.append(
                self.create_subscription(
                    message_type,
                    topic,
                    lambda msg, topic=topic: self.on_message(topic, msg),
                    qos,
                )
            )

    def on_message(self, topic, msg):
        now = time.monotonic()
        record = self.records[topic]
        record["receive"].append(now)
        if topic == "/clock":
            self.clock_stamp = stamp_seconds(msg.clock)
            record["stamp"].append(self.clock_stamp)
            return
        header = getattr(msg, "header", None)
        if header is not None:
            message_stamp = stamp_seconds(header.stamp)
            record["stamp"].append(message_stamp)
            if self.clock_stamp is not None:
                record["age"].append(self.clock_stamp - message_stamp)
            record["frame_ids"].add(header.frame_id)
        if topic == "/human_states":
            record["agent_counts"].add(len(msg.agents))
            record["agent_ids"].update(int(agent.id) for agent in msg.agents)

    def endpoint_snapshot(self):
        snapshot = {}
        for topic in TOPICS:
            endpoints = []
            for endpoint in self.get_publishers_info_by_topic(topic):
                qos = endpoint.qos_profile
                endpoints.append(
                    {
                        "node": f"{endpoint.node_namespace.rstrip('/')}/{endpoint.node_name}",
                        "topic_type": endpoint.topic_type,
                        "reliability": qos.reliability.name,
                        "durability": qos.durability.name,
                        "history": qos.history.name,
                        "depth": qos.depth,
                    }
                )
            snapshot[topic] = endpoints
        return snapshot

    def report(self, duration):
        topics = {}
        for topic, record in self.records.items():
            receives = record["receive"]
            stamps = record["stamp"]
            receive_intervals = [b - a for a, b in zip(receives, receives[1:])]
            stamp_intervals = [b - a for a, b in zip(stamps, stamps[1:])]
            non_monotonic = sum(delta <= 0.0 for delta in stamp_intervals)
            age_samples = record["age"]
            topics[topic] = {
                "count": len(receives),
                "wall_hz": len(receives) / duration,
                "wall_interval_p50_s": statistics.median(receive_intervals)
                if receive_intervals
                else None,
                "wall_interval_p95_s": quantile(receive_intervals, 0.95),
                "wall_interval_p99_s": quantile(receive_intervals, 0.99),
                "stamp_interval_p50_s": statistics.median(stamp_intervals)
                if stamp_intervals
                else None,
                "stamp_non_monotonic": non_monotonic,
                "clock_minus_stamp_p50_s": statistics.median(age_samples)
                if age_samples
                else None,
                "clock_minus_stamp_p95_s": quantile(age_samples, 0.95),
                "clock_minus_stamp_p99_s": quantile(age_samples, 0.99),
                "frame_ids": sorted(record["frame_ids"]),
                "agent_ids": sorted(record["agent_ids"]),
                "agent_counts": sorted(record["agent_counts"]),
            }
        return {
            "duration_wall_s": duration,
            "endpoints": self.endpoint_snapshot(),
            "topics": topics,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=60.0)
    args = parser.parse_args()
    rclpy.init(args=None)
    observer = Observer()
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < args.duration:
            rclpy.spin_once(observer, timeout_sec=0.1)
        elapsed = time.monotonic() - started
        print(json.dumps(observer.report(elapsed), indent=2, sort_keys=True))
    finally:
        observer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
