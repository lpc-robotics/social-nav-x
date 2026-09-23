#!/usr/bin/env python3
"""Measure whether an ideal D6 chassis remains collision-limited by PhysX."""

import argparse
import csv
import json
import math
import statistics
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def roll_pitch_from_quaternion(q):
    roll = math.atan2(
        2.0 * (q.w * q.x + q.y * q.z),
        1.0 - 2.0 * (q.x * q.x + q.y * q.y),
    )
    sin_pitch = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    return roll, pitch


def angle_delta(new: float, old: float) -> float:
    return math.atan2(math.sin(new - old), math.cos(new - old))


def mean(values):
    return statistics.fmean(values) if values else None


def median(values):
    return statistics.median(values) if values else None


class CollisionNode(Node):
    def __init__(self):
        super().__init__("ideal_chassis_collision")
        self.command_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(TFMessage, "/tf", self._on_tf, 50)
        self.samples = []
        self.last_pose = None
        self.last_stamp_ns = None
        self.last_tf_wall = None

    def publish_command(self, linear: float, angular: float):
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self.command_pub.publish(command)

    def _on_tf(self, message: TFMessage):
        for transform in message.transforms:
            if (
                transform.header.frame_id.lstrip("/") != "odom"
                or transform.child_frame_id.lstrip("/") != "base_link"
            ):
                continue
            wall_time = time.monotonic()
            stamp_ns = (
                transform.header.stamp.sec * 1_000_000_000
                + transform.header.stamp.nanosec
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            yaw = yaw_from_quaternion(rotation)
            roll, pitch = roll_pitch_from_quaternion(rotation)
            world_vx = world_vy = wz = 0.0
            if self.last_pose is not None and stamp_ns > self.last_stamp_ns:
                dt = (stamp_ns - self.last_stamp_ns) * 1e-9
                if 0.001 <= dt <= 0.25:
                    old_x, old_y, old_yaw = self.last_pose
                    world_vx = (translation.x - old_x) / dt
                    world_vy = (translation.y - old_y) / dt
                    wz = angle_delta(yaw, old_yaw) / dt
                    self.samples.append(
                        (
                            wall_time,
                            translation.x,
                            translation.y,
                            translation.z,
                            yaw,
                            roll,
                            pitch,
                            world_vx,
                            world_vy,
                            wz,
                        )
                    )
            self.last_pose = (translation.x, translation.y, yaw)
            self.last_stamp_ns = stamp_ns
            self.last_tf_wall = wall_time


def drive(node: CollisionNode, linear: float, angular: float, duration: float):
    end = time.monotonic() + duration
    next_publish = 0.0
    while rclpy.ok() and time.monotonic() < end:
        now = time.monotonic()
        if now >= next_publish:
            node.publish_command(linear, angular)
            next_publish = now + 0.04
        rclpy.spin_once(node, timeout_sec=0.01)


def wall_values(wall: str, sample, cmd_v: float):
    _, x, y, _, yaw, _, _, world_vx, world_vy, _ = sample
    cmd_world_vx = math.cos(yaw) * cmd_v
    cmd_world_vy = math.sin(yaw) * cmd_v
    if wall == "left":
        return x, -world_vx, -cmd_world_vx
    if wall == "right":
        return 30.0 - x, world_vx, cmd_world_vx
    if wall == "bottom":
        return y, -world_vy, -cmd_world_vy
    return 23.0 - y, world_vy, cmd_world_vy


def summarize(label: str, wall: str, cmd_v: float, cmd_w: float, samples):
    clearances = [wall_values(wall, sample, cmd_v)[0] for sample in samples]
    contact_samples = [
        (*wall_values(wall, sample, cmd_v), sample)
        for sample in samples
        if wall_values(wall, sample, cmd_v)[0] < 0.5
        and wall_values(wall, sample, cmd_v)[2] > 0.05
    ]
    actual_toward = [max(0.0, item[1]) for item in contact_samples]
    commanded_toward = [item[2] for item in contact_samples]
    ratios = [
        actual / command
        for actual, command in zip(actual_toward, commanded_toward)
        if command > 1e-6
    ]
    roll = [abs(sample[5]) for sample in samples]
    pitch = [abs(sample[6]) for sample in samples]
    z = [sample[3] for sample in samples]
    min_clearance = min(clearances, default=None)
    contact_detected = bool(contact_samples)
    no_center_penetration = min_clearance is not None and min_clearance >= 0.20
    posture_valid = max(roll, default=0.0) < 0.15 and max(
        pitch, default=0.0
    ) < 0.15
    final = samples[-1] if samples else (None,) * 10
    return {
        "label": label,
        "wall": wall,
        "cmd_v": cmd_v,
        "cmd_w": cmd_w,
        "samples": len(samples),
        "contact_samples": len(contact_samples),
        "contact_detected": contact_detected,
        "min_clearance": min_clearance,
        "final_x": final[1],
        "final_y": final[2],
        "final_yaw": final[4],
        "contact_actual_toward_mean": mean(actual_toward),
        "contact_actual_toward_median": median(actual_toward),
        "contact_commanded_toward_mean": mean(commanded_toward),
        "collision_limited_ratio_mean": mean(ratios),
        "max_abs_roll": max(roll, default=None),
        "max_abs_pitch": max(pitch, default=None),
        "min_base_z": min(z, default=None),
        "max_base_z": max(z, default=None),
        "no_center_penetration": no_center_penetration,
        "posture_valid": posture_valid,
        "valid": contact_detected and no_center_penetration and posture_valid,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--wall", choices=("left", "right", "bottom", "top"), required=True)
    parser.add_argument("--linear", type=float, required=True)
    parser.add_argument("--angular", type=float, required=True)
    parser.add_argument("--settle", type=float, default=1.0)
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--append", action="store_true")
    args = parser.parse_args()

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = CollisionNode()
    try:
        deadline = time.monotonic() + 30.0
        while rclpy.ok() and node.last_tf_wall is None and time.monotonic() < deadline:
            node.publish_command(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.last_tf_wall is None:
            raise RuntimeError("no odom->base_link TF received")
        drive(node, 0.0, 0.0, args.settle)
        start = time.monotonic()
        drive(node, args.linear, args.angular, args.duration)
        samples = [sample for sample in node.samples if sample[0] >= start]
        result = summarize(
            args.label, args.wall, args.linear, args.angular, samples
        )
        drive(node, 0.0, 0.0, 1.0)

        json_path = output / "results.json"
        combined = [result]
        if args.append and json_path.exists():
            with json_path.open("r", encoding="utf-8") as stream:
                combined = [*json.load(stream), result]
        with json_path.open("w", encoding="utf-8") as stream:
            json.dump(combined, stream, indent=2)
        with (output / "results.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(combined[0]))
            writer.writeheader()
            writer.writerows(combined)
        print(
            "COLLISION_RESULT "
            f"label={args.label} valid={result['valid']} "
            f"min_clearance={result['min_clearance']:.4f} "
            f"limited_ratio={result['collision_limited_ratio_mean']:.4f} "
            f"final=({result['final_x']:.4f},{result['final_y']:.4f},"
            f"{result['final_yaw']:.4f})",
            flush=True,
        )
    finally:
        try:
            drive(node, 0.0, 0.0, 0.5)
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
