"""Audit a live normalized LaserScan without changing runtime state."""

import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", action="append", default=[])
    parser.add_argument("--messages", type=int, default=25)
    parser.add_argument("--timeout", type=float, default=45.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--expect-offset-z", type=float)
    args = parser.parse_args()
    topics = args.topic or ["/lidar_normalized"]

    rclpy.init()
    node = Node("audit_normalized_scan", use_global_arguments=False)
    received = {topic: [] for topic in topics}
    static_transforms = []

    def callback(topic, message):
        values = np.asarray(message.ranges, dtype=np.float32)
        hit = np.isfinite(values) & (values >= message.range_min) & (values <= message.range_max)
        record = {
            "arrival_monotonic": time.monotonic(),
            "stamp": message.header.stamp.sec + message.header.stamp.nanosec / 1e9,
            "frame_id": message.header.frame_id,
            "beams": len(values),
            "angle_min": message.angle_min,
            "angle_max": message.angle_max,
            "angle_increment": message.angle_increment,
            "time_increment": message.time_increment,
            "scan_time": message.scan_time,
            "range_min": message.range_min,
            "range_max": message.range_max,
            "hit": int(hit.sum()),
            "no_return": int(np.isposinf(values).sum()),
            "too_close": int(np.isneginf(values).sum()),
            "unknown": int((~(hit | np.isposinf(values) | np.isneginf(values))).sum()),
            "invalid_finite": int((np.isfinite(values) & ~hit).sum()),
        }
        received[topic].append(record)

    subscriptions = []
    for topic in topics:
        subscriptions.append(
            node.create_subscription(
                LaserScan,
                topic,
                lambda message, selected=topic: callback(selected, message),
                qos_profile_sensor_data,
            )
        )
    tf_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    subscriptions.append(
        node.create_subscription(
            TFMessage,
            "/tf_static",
            lambda message: static_transforms.extend(message.transforms),
            tf_qos,
        )
    )

    deadline = time.monotonic() + args.timeout
    while (
        any(len(items) < args.messages for items in received.values())
        and time.monotonic() < deadline
    ):
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    summaries = {}
    for topic, records in received.items():
        assert len(records) >= args.messages, f"{topic}: only received {len(records)} messages"
        records = records[:args.messages]
        beams = {record["beams"] for record in records}
        frames = {record["frame_id"] for record in records}
        stamps = np.asarray([record["stamp"] for record in records])
        periods = np.diff(stamps)
        assert len(beams) == 1 and next(iter(beams)) > 0
        assert len(frames) == 1 and next(iter(frames))
        assert np.all(periods > 0.0)
        assert all(record["invalid_finite"] == 0 for record in records)
        assert all(record["time_increment"] == 0.0 for record in records)
        assert all(
            math.isclose(
                record["angle_max"],
                record["angle_min"] + (record["beams"] - 1) * record["angle_increment"],
                abs_tol=1e-5,
            )
            for record in records
        )
        summaries[topic] = {
            "messages": len(records),
            "beams": next(iter(beams)),
            "frame_id": next(iter(frames)),
            "sim_frequency_hz": float(1.0 / np.median(periods)),
            "states_last": {
                key: records[-1][key] for key in ("hit", "no_return", "too_close", "unknown")
            },
            "records": records,
        }

        child_frame = next(iter(frames))
        transforms = [
            transform
            for transform in static_transforms
            if transform.child_frame_id == child_frame
        ]
        assert transforms, f"missing static transform for {child_frame}"
        transform = transforms[-1]
        offset = transform.transform.translation
        rotation = transform.transform.rotation
        if args.expect_offset_z is not None:
            assert math.isclose(offset.z, args.expect_offset_z, abs_tol=1e-9)
        summaries[topic]["static_transform"] = {
            "parent_frame_id": transform.header.frame_id,
            "child_frame_id": transform.child_frame_id,
            "translation": {"x": offset.x, "y": offset.y, "z": offset.z},
            "rotation": {
                "x": rotation.x,
                "y": rotation.y,
                "z": rotation.z,
                "w": rotation.w,
            },
        }

    result = {"passed": True, "topics": summaries}
    output = args.output or Path(".workspaces/laserscan-v1/log/live_normalized_scan.json")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2))
    print(json.dumps({"passed": True, "topics": list(summaries)}, indent=2))


if __name__ == "__main__":
    main()
