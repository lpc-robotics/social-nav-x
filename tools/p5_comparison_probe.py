#!/usr/bin/env python3
"""Read-only, controller-neutral probe for paired DWB/MPC runs."""

import argparse
import bisect
import json
import math
import statistics
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from hunav_msgs.msg import Agents
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


ROBOT_HALF_LENGTH = 0.24
ROBOT_HALF_WIDTH = 0.22
ROBOT_CORNER_RADIUS = math.hypot(ROBOT_HALF_LENGTH, ROBOT_HALF_WIDTH)
ROBOT_LINEAR_LIMIT = 0.26
ROBOT_ANGULAR_LIMIT = 1.0
NUMERIC_ALLOWANCE = 0.005


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def yaw_of(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def footprint_clearance(robot, human):
    x, y, yaw = robot
    dx = human[1] - x
    dy = human[2] - y
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    outside_x = max(abs(local_x) - ROBOT_HALF_LENGTH, 0.0)
    outside_y = max(abs(local_y) - ROBOT_HALF_WIDTH, 0.0)
    point_clearance = (
        math.hypot(outside_x, outside_y)
        if outside_x > 0.0 or outside_y > 0.0
        else -min(
            ROBOT_HALF_LENGTH - abs(local_x), ROBOT_HALF_WIDTH - abs(local_y)
        )
    )
    return point_clearance - human[3]


class ComparisonProbe(Node):
    def __init__(self, args):
        super().__init__(f"p5_{args.method}_{args.pair}_probe")
        self.args = args
        self.navigation = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.parameters = self.create_client(
            GetParameters, "/controller_server/get_parameters"
        )
        self.local_costmap_parameters = self.create_client(
            GetParameters, "/local_costmap/local_costmap/get_parameters"
        )
        self.global_costmap_parameters = self.create_client(
            GetParameters, "/global_costmap/global_costmap/get_parameters"
        )
        self.odom = []
        self.humans = []
        self.commands = []
        self.lidar_count = 0
        self.latest_odom = None
        self.latest_humans = None
        self.create_subscription(Odometry, "/odom", self.on_odom, 50)
        self.create_subscription(Agents, "/human_states", self.on_humans, 20)
        self.create_subscription(Twist, "/cmd_vel", self.on_command, 50)
        self.create_subscription(LaserScan, "/lidar", self.on_lidar, 20)

    def on_odom(self, message):
        self.latest_odom = message
        pose = message.pose.pose
        self.odom.append(
            (
                stamp_ns(message.header.stamp),
                pose.position.x,
                pose.position.y,
                yaw_of(pose.orientation),
            )
        )

    def on_humans(self, message):
        self.latest_humans = message
        self.humans.append(
            (
                stamp_ns(message.header.stamp),
                [
                    (
                        agent.id,
                        agent.position.position.x,
                        agent.position.position.y,
                        agent.radius,
                        agent.velocity.linear.x,
                        agent.velocity.linear.y,
                        agent.desired_velocity,
                    )
                    for agent in message.agents
                ],
            )
        )

    def on_command(self, message):
        self.commands.append((time.monotonic(), message.linear.x, message.angular.z))

    def on_lidar(self, _message):
        self.lidar_count += 1

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return True
        return False

    def controller_parameters(self):
        if not self.parameters.wait_for_service(timeout_sec=60.0):
            raise RuntimeError("controller parameter service unavailable")
        request = GetParameters.Request()
        speed_parameter = (
            "FollowPath.max_vel_x"
            if self.args.method == "dwb"
            else "FollowPath.max_linear"
        )
        request.names = [
            "controller_plugins",
            "FollowPath.plugin",
            speed_parameter,
        ]
        future = self.parameters.call_async(request)
        if not self.spin_until(future.done, 5.0):
            raise RuntimeError("controller parameter request timed out")
        values = future.result().values
        if len(values) != len(request.names):
            raise RuntimeError(
                "controller parameter response length mismatch: "
                f"requested {len(request.names)}, received {len(values)}"
            )
        return {
            "controller_plugins": list(values[0].string_array_value),
            "follow_path_plugin": values[1].string_value,
            "speed_parameter": speed_parameter,
            "configured_max_linear_mps": values[2].double_value,
        }

    def costmap_parameters(self):
        result = {}
        for label, client in (
            ("local", self.local_costmap_parameters),
            ("global", self.global_costmap_parameters),
        ):
            if not client.wait_for_service(timeout_sec=60.0):
                raise RuntimeError(f"{label} costmap parameter service unavailable")
            request = GetParameters.Request()
            request.names = ["plugins", "footprint"]
            future = client.call_async(request)
            if not self.spin_until(future.done, 5.0):
                raise RuntimeError(f"{label} costmap parameter request timed out")
            values = future.result().values
            if len(values) != len(request.names):
                raise RuntimeError(
                    f"{label} costmap parameter response length mismatch: "
                    f"requested {len(request.names)}, received {len(values)}"
                )
            result[label] = {
                "plugins": list(values[0].string_array_value),
                "footprint": values[1].string_value,
            }
        return result

    @staticmethod
    def interpolate(samples, target_ns):
        stamps = [item[0] for item in samples]
        upper = bisect.bisect_left(stamps, target_ns)
        if upper == 0 or upper >= len(samples):
            return None
        first = samples[upper - 1]
        second = samples[upper]
        interval = second[0] - first[0]
        if interval <= 0:
            return None
        ratio = (target_ns - first[0]) / interval
        return (
            first[1] + ratio * (second[1] - first[1]),
            first[2] + ratio * (second[2] - first[2]),
            first[3] + ratio * wrap(second[3] - first[3]),
            interval * 1.0e-9,
        )

    def safety(self, start_ns, end_ns):
        odom = sorted({sample[0]: sample for sample in self.odom}.values())
        humans = sorted(
            sample for sample in self.humans if start_ns <= sample[0] <= end_ns
        )
        clearances = []
        odom_gaps = []
        human_speeds = []
        closest_by_id = {}
        for human_stamp, agents in humans:
            robot = self.interpolate(odom, human_stamp)
            if robot is None:
                continue
            odom_gaps.append(robot[3])
            for human in agents:
                clearance = footprint_clearance(robot[:3], human)
                clearances.append(clearance)
                closest_by_id[human[0]] = min(
                    closest_by_id.get(human[0], math.inf), clearance
                )
                human_speeds.append(
                    max(math.hypot(human[4], human[5]), human[6])
                )
        human_stamps = sorted(set(sample[0] for sample in humans))
        coverage = [start_ns] + human_stamps + [end_ns]
        max_human_gap = max(
            ((b - a) * 1e-9 for a, b in zip(coverage, coverage[1:]) if b >= a),
            default=math.inf,
        )
        max_odom_gap = max(odom_gaps, default=math.inf)
        corner_speed = ROBOT_LINEAR_LIMIT + ROBOT_CORNER_RADIUS * ROBOT_ANGULAR_LIMIT
        error_bound = (
            0.5 * (corner_speed + max(human_speeds, default=0.0)) * max_human_gap
            + 0.5 * corner_speed * max_odom_gap
            + NUMERIC_ALLOWANCE
        )
        measured = min(clearances, default=None)
        return {
            "aligned_human_samples": len(humans),
            "minimum_measured_footprint_human_clearance_m": measured,
            "sampling_alignment_error_bound_m": error_bound,
            "minimum_clearance_lower_bound_m": (
                measured - error_bound if measured is not None else None
            ),
            "minimum_measured_clearance_by_id_m": {
                str(key): value for key, value in sorted(closest_by_id.items())
            },
            "maximum_human_stamp_gap_s": max_human_gap,
            "maximum_odom_bracket_s": max_odom_gap,
        }

    def run(self):
        if not self.spin_until(
            lambda: self.latest_odom is not None
            and self.latest_humans is not None
            and len(self.latest_humans.agents) == self.args.expected_agents
            and self.lidar_count >= 3,
            120.0,
        ):
            raise RuntimeError("timed out waiting for odom, lidar, and HuNav")
        if not self.navigation.wait_for_server(timeout_sec=60.0):
            raise RuntimeError("navigate_to_pose server unavailable")
        parameters = self.controller_parameters()
        costmaps = self.costmap_parameters()
        expected_token = "DWB" if self.args.method == "dwb" else "MpcController"
        plugin_matches = expected_token in parameters["follow_path_plugin"]
        speed_matches = abs(parameters["configured_max_linear_mps"] - 0.26) <= 1e-9
        expected_footprint = "[[0.24,0.22],[0.24,-0.22],[-0.24,-0.22],[-0.24,0.22]]"
        mpc_costmap_matches = self.args.method == "dwb" or (
            costmaps["global"]["plugins"]
            == ["static_layer", "obstacle_layer", "inflation_layer"]
            and "".join(costmaps["local"]["footprint"].split()) == expected_footprint
            and "".join(costmaps["global"]["footprint"].split()) == expected_footprint
        )

        start_index = len(self.odom)
        command_index = len(self.commands)
        start_ns = stamp_ns(self.latest_odom.header.stamp)
        start_wall = time.monotonic()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.args.target_x
        goal.pose.pose.position.y = self.args.target_y
        goal.pose.pose.orientation.z = math.sin(self.args.target_yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(self.args.target_yaw * 0.5)
        future = self.navigation.send_goal_async(goal)
        if not self.spin_until(future.done, 30.0):
            raise RuntimeError("goal acceptance timed out")
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("goal rejected")
        result = handle.get_result_async()
        if not self.spin_until(result.done, self.args.timeout):
            handle.cancel_goal_async()
            raise RuntimeError("navigation timed out")
        status = result.result().status
        self.spin_until(lambda: False, 0.35)
        end_ns = stamp_ns(self.latest_odom.header.stamp)
        pose = self.latest_odom.pose.pose
        position_error = math.hypot(
            pose.position.x - self.args.target_x,
            pose.position.y - self.args.target_y,
        )
        yaw_error = abs(wrap(yaw_of(pose.orientation) - self.args.target_yaw))
        action_odom = self.odom[start_index:]
        path_length = sum(
            math.hypot(second[1] - first[1], second[2] - first[2])
            for first, second in zip(action_odom, action_odom[1:])
        )
        commands = self.commands[command_index:]
        total_variation = sum(
            abs(second[1] - first[1]) + abs(second[2] - first[2])
            for first, second in zip(commands, commands[1:])
        )
        finite = all(
            math.isfinite(item[1]) and math.isfinite(item[2]) for item in commands
        )
        safety = self.safety(start_ns, end_ns)
        lower = safety["minimum_clearance_lower_bound_m"]
        measured = safety["minimum_measured_footprint_human_clearance_m"]
        passed = (
            status == GoalStatus.STATUS_SUCCEEDED
            and plugin_matches
            and speed_matches
            and mpc_costmap_matches
            and position_error <= 0.25
            and yaw_error <= 0.25
            and finite
            and commands
            and measured is not None
            and measured <= self.args.interaction_distance
            and safety["sampling_alignment_error_bound_m"] <= 0.05
            and lower is not None
            and (self.args.method == "dwb" or lower >= 0.30)
        )
        return {
            "method": self.args.method,
            "pair": self.args.pair,
            "order": self.args.order,
            "action_status": status,
            "pass": bool(passed),
            "controller": parameters,
            "costmaps": costmaps,
            "plugin_matches_method": plugin_matches,
            "speed_limit_matches_pairing": speed_matches,
            "mpc_costmap_gate": mpc_costmap_matches,
            "target": [self.args.target_x, self.args.target_y, self.args.target_yaw],
            "end": [pose.position.x, pose.position.y, yaw_of(pose.orientation)],
            "position_error_m": position_error,
            "yaw_error_rad": yaw_error,
            "simulation_duration_s": (end_ns - start_ns) * 1e-9,
            "wall_duration_s": time.monotonic() - start_wall,
            "path_length_m": path_length,
            "command_samples": len(commands),
            "maximum_abs_linear_command_mps": max(
                (abs(item[1]) for item in commands), default=0.0
            ),
            "maximum_abs_angular_command_rps": max(
                (abs(item[2]) for item in commands), default=0.0
            ),
            "command_total_variation": total_variation,
            "outputs_finite": finite,
            "lidar_samples": self.lidar_count,
            "safety": safety,
            "run_metadata": {
                "ros_domain_id": self.args.ros_domain_id,
                "gpu_index": self.args.gpu_index,
                "gpu_snapshot_csv": self.args.gpu_snapshot,
                "ideal_chassis": self.args.ideal_chassis,
                "physics_dt_s": self.args.physics_dt,
                "config_path": self.args.config_path,
                "config_sha256": self.args.config_sha256,
                "probe_sha256": self.args.probe_sha256,
                "source_revision": self.args.source_revision,
                "launch_log": self.args.launch_log,
            },
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("method", choices=("dwb", "mpc"))
    parser.add_argument("--pair", type=int, required=True)
    parser.add_argument("--order", type=int, choices=(1, 2), required=True)
    parser.add_argument("--target-x", type=float, default=3.6)
    parser.add_argument("--target-y", type=float, default=3.0)
    parser.add_argument("--target-yaw", type=float, default=0.0)
    parser.add_argument("--expected-agents", type=int, default=6)
    parser.add_argument("--interaction-distance", type=float, default=1.5)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ros-domain-id", type=int, required=True)
    parser.add_argument("--gpu-index", type=int, required=True)
    parser.add_argument("--gpu-snapshot", required=True)
    parser.add_argument("--ideal-chassis", required=True)
    parser.add_argument("--physics-dt", type=float, required=True)
    parser.add_argument("--config-path", required=True)
    parser.add_argument("--config-sha256", required=True)
    parser.add_argument("--probe-sha256", required=True)
    parser.add_argument("--source-revision", required=True)
    parser.add_argument("--launch-log", required=True)
    args = parser.parse_args()

    rclpy.init()
    probe = ComparisonProbe(args)
    exit_code = 0
    try:
        report = probe.run()
        if not report["pass"]:
            exit_code = 2
    except Exception as error:
        report = {
            "method": args.method,
            "pair": args.pair,
            "order": args.order,
            "error": str(error),
            "pass": False,
        }
        exit_code = 1
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    print(rendered, end="")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(rendered, encoding="utf-8")
    probe.destroy_node()
    rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
