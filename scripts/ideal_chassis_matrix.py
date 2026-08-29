#!/usr/bin/env python3
"""Measure chassis tracking from the actual odom->base_link TF trajectory."""

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
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


WHEEL_JOINTS = (
    "front_left_wheel_joint",
    "rear_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_right_wheel_joint",
)

CASES = (
    (0.2, 0.0),
    (0.5, 0.0),
    (0.8, 0.0),
    (0.0, 0.2),
    (0.0, -0.2),
    (0.0, 0.4),
    (0.0, -0.4),
    (0.0, 0.8),
    (0.0, -0.8),
    (0.0, 1.2),
    (0.3, 0.4),
    (0.3, -0.4),
    (0.3, 0.8),
    (0.3, -0.8),
    (0.6, 0.4),
    (0.6, -0.4),
    (0.6, 0.8),
    (0.6, -0.8),
    (0.45, 0.6),
    (0.45, -0.6),
    (0.2, 0.6),
    (0.7, -0.6),
)


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


def stddev(values):
    return statistics.pstdev(values) if values else None


def relative_error(actual, command):
    if actual is None or abs(command) < 1e-9:
        return None
    return 100.0 * abs(actual - command) / abs(command)


def sample_rate(times):
    if len(times) < 2 or times[-1] <= times[0]:
        return None
    return (len(times) - 1) / (times[-1] - times[0])


class MatrixNode(Node):
    def __init__(self):
        super().__init__("ideal_chassis_matrix")
        self.command_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(TFMessage, "/tf", self._on_tf, 50)
        self.create_subscription(
            JointState,
            "/isaac/joint_commands_velocity",
            self._on_wheel_command,
            20,
        )
        self.create_subscription(
            JointState, "/isaac/joint_states", self._on_wheel_actual, 20
        )
        self.twist_samples = []
        self.wheel_command_samples = []
        self.wheel_actual_samples = []
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
            current = (translation.x, translation.y, translation.z, yaw)
            if self.last_pose is not None and stamp_ns > self.last_stamp_ns:
                dt = (stamp_ns - self.last_stamp_ns) * 1e-9
                if 0.001 <= dt <= 0.25:
                    old_x, old_y, _, old_yaw = self.last_pose
                    dx = translation.x - old_x
                    dy = translation.y - old_y
                    dyaw = angle_delta(yaw, old_yaw)
                    mid_yaw = old_yaw + 0.5 * dyaw
                    body_vx = (
                        math.cos(mid_yaw) * dx + math.sin(mid_yaw) * dy
                    ) / dt
                    body_vy = (
                        -math.sin(mid_yaw) * dx + math.cos(mid_yaw) * dy
                    ) / dt
                    roll, pitch = roll_pitch_from_quaternion(rotation)
                    self.twist_samples.append(
                        (
                            wall_time,
                            body_vx,
                            body_vy,
                            dyaw / dt,
                            roll,
                            pitch,
                            translation.z,
                            stamp_ns * 1e-9,
                            translation.x,
                            translation.y,
                        )
                    )
            self.last_pose = current
            self.last_stamp_ns = stamp_ns
            self.last_tf_wall = wall_time

    @staticmethod
    def _wheel_values(message: JointState):
        if not message.name or not message.velocity:
            return None
        values = dict(zip(message.name, message.velocity))
        if not all(name in values for name in WHEEL_JOINTS):
            return None
        left = 0.5 * (values[WHEEL_JOINTS[0]] + values[WHEEL_JOINTS[1]])
        right = 0.5 * (values[WHEEL_JOINTS[2]] + values[WHEEL_JOINTS[3]])
        return left, right

    @staticmethod
    def _wheel_efforts(message: JointState):
        if not message.name or not message.effort:
            return None, None
        values = dict(zip(message.name, message.effort))
        if not all(name in values for name in WHEEL_JOINTS):
            return None, None
        left = 0.5 * (values[WHEEL_JOINTS[0]] + values[WHEEL_JOINTS[1]])
        right = 0.5 * (values[WHEEL_JOINTS[2]] + values[WHEEL_JOINTS[3]])
        return left, right

    def _on_wheel_command(self, message: JointState):
        values = self._wheel_values(message)
        if values is not None:
            self.wheel_command_samples.append((time.monotonic(), *values))

    def _on_wheel_actual(self, message: JointState):
        values = self._wheel_values(message)
        if values is not None:
            self.wheel_actual_samples.append(
                (time.monotonic(), *values, *self._wheel_efforts(message))
            )


def drive(node: MatrixNode, linear: float, angular: float, duration: float):
    end = time.monotonic() + duration
    next_publish = 0.0
    while rclpy.ok() and time.monotonic() < end:
        now = time.monotonic()
        if now >= next_publish:
            node.publish_command(linear, angular)
            next_publish = now + 0.04
        rclpy.spin_once(node, timeout_sec=0.01)


def summarize(node: MatrixNode, cmd_v: float, cmd_w: float, start: float):
    chassis = [sample for sample in node.twist_samples if sample[0] >= start]
    wheel_cmd = [
        sample for sample in node.wheel_command_samples if sample[0] >= start
    ]
    wheel_actual = [
        sample for sample in node.wheel_actual_samples if sample[0] >= start
    ]
    vx = [sample[1] for sample in chassis]
    vy = [sample[2] for sample in chassis]
    wz = [sample[3] for sample in chassis]
    roll = [sample[4] for sample in chassis]
    pitch = [sample[5] for sample in chassis]
    z = [sample[6] for sample in chassis]
    sim_times = [sample[7] for sample in chassis]
    x = [sample[8] for sample in chassis]
    y = [sample[9] for sample in chassis]
    cmd_left = [sample[1] for sample in wheel_cmd]
    cmd_right = [sample[2] for sample in wheel_cmd]
    actual_left = [sample[1] for sample in wheel_actual]
    actual_right = [sample[2] for sample in wheel_actual]
    effort_left = [sample[3] for sample in wheel_actual if sample[3] is not None]
    effort_right = [sample[4] for sample in wheel_actual if sample[4] is not None]
    max_roll = max((abs(value) for value in roll), default=None)
    max_pitch = max((abs(value) for value in pitch), default=None)
    invalid_reasons = []
    if any(
        px < 0.4 or px > 29.6 or py < 0.4 or py > 22.6
        for px, py in zip(x, y)
    ):
        invalid_reasons.append("arena_boundary_contact")
    if (max_roll is not None and max_roll > 0.15) or (
        max_pitch is not None and max_pitch > 0.15
    ):
        invalid_reasons.append("tilt")

    actual_v_mean = mean(vx)
    actual_w_mean = mean(wz)
    return {
        "cmd_v": cmd_v,
        "cmd_w": cmd_w,
        "actual_v_mean": actual_v_mean,
        "actual_w_mean": actual_w_mean,
        "actual_v_median": median(vx),
        "actual_w_median": median(wz),
        "actual_v_std": stddev(vx),
        "actual_w_std": stddev(wz),
        "error_v": None if actual_v_mean is None else abs(actual_v_mean - cmd_v),
        "error_w": None if actual_w_mean is None else abs(actual_w_mean - cmd_w),
        "error_v_pct": relative_error(actual_v_mean, cmd_v),
        "error_w_pct": relative_error(actual_w_mean, cmd_w),
        "median_error_v_pct": relative_error(median(vx), cmd_v),
        "median_error_w_pct": relative_error(median(wz), cmd_w),
        "actual_vy_mean": mean(vy),
        "max_abs_roll": max_roll,
        "max_abs_pitch": max_pitch,
        "min_base_z": min(z, default=None),
        "max_base_z": max(z, default=None),
        "valid": not invalid_reasons,
        "invalid_reason": ",".join(invalid_reasons),
        "wheel_cmd_left_mean": mean(cmd_left),
        "wheel_cmd_right_mean": mean(cmd_right),
        "wheel_actual_left_mean": mean(actual_left),
        "wheel_actual_right_mean": mean(actual_right),
        "wheel_effort_left_mean": mean(effort_left),
        "wheel_effort_right_mean": mean(effort_right),
        "wheel_effort_left_max_abs": max(
            (abs(value) for value in effort_left), default=None
        ),
        "wheel_effort_right_max_abs": max(
            (abs(value) for value in effort_right), default=None
        ),
        "odom_samples": len(chassis),
        "odom_wall_rate_hz": sample_rate([sample[0] for sample in chassis]),
        "odom_sim_rate_hz": sample_rate(sim_times),
        "controller_wall_rate_hz": sample_rate(
            [sample[0] for sample in chassis]
        ),
        "wheel_command_samples": len(wheel_cmd),
        "wheel_actual_samples": len(wheel_actual),
    }


def parse_case(value: str):
    try:
        linear, angular = value.split(",", maxsplit=1)
        return float(linear), float(angular)
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            "case must be formatted as linear,angular"
        ) from error


def fmt(value):
    return "n/a" if value is None else f"{value:.4f}"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--settle", type=float, default=1.0)
    parser.add_argument("--warmup", type=float, default=1.5)
    parser.add_argument("--measure", type=float, default=3.0)
    parser.add_argument("--case", action="append", type=parse_case, dest="cases")
    parser.add_argument(
        "--append",
        action="store_true",
        help="append this process' cases to an existing results.json/results.csv",
    )
    args = parser.parse_args()
    cases = tuple(args.cases) if args.cases else CASES
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = MatrixNode()
    results = []
    try:
        deadline = time.monotonic() + 30.0
        while rclpy.ok() and node.last_tf_wall is None and time.monotonic() < deadline:
            node.publish_command(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.last_tf_wall is None:
            raise RuntimeError("no odom->base_link TF received")

        for index, (cmd_v, cmd_w) in enumerate(cases, start=1):
            drive(node, 0.0, 0.0, args.settle)
            drive(node, cmd_v, cmd_w, args.warmup)
            measure_start = time.monotonic()
            drive(node, cmd_v, cmd_w, args.measure)
            result = summarize(node, cmd_v, cmd_w, measure_start)
            results.append(result)
            print(
                f"MATRIX_CASE {index:02d}/{len(cases)} cmd=({cmd_v:+.2f},{cmd_w:+.2f}) "
                f"actual=({fmt(result['actual_v_mean'])},{fmt(result['actual_w_mean'])}) "
                f"error_pct=({fmt(result['error_v_pct'])},{fmt(result['error_w_pct'])}) "
                f"valid={result['valid']} samples={result['odom_samples']}",
                flush=True,
            )
        drive(node, 0.0, 0.0, 1.0)

        combined_results = results
        json_path = output / "results.json"
        if args.append and json_path.exists():
            with json_path.open("r", encoding="utf-8") as stream:
                existing_results = json.load(stream)
            if not isinstance(existing_results, list):
                raise RuntimeError(f"{json_path} does not contain a result list")
            combined_results = [*existing_results, *results]

        with json_path.open("w", encoding="utf-8") as stream:
            json.dump(combined_results, stream, indent=2)
        with (output / "results.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(combined_results[0]))
            writer.writeheader()
            writer.writerows(combined_results)
        print(
            f"MATRIX_COMPLETE output={output} "
            f"new_cases={len(results)} total_cases={len(combined_results)}",
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
