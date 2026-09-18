#!/usr/bin/env python3
import argparse
import bisect
import copy
import json
import math
import re
import statistics
import time
from collections import Counter
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from hunav_msgs.msg import Agents
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String


ROBOT_HALF_LENGTH = 0.24
ROBOT_HALF_WIDTH = 0.22
ROBOT_CIRCUMSCRIBED_RADIUS = math.hypot(ROBOT_HALF_LENGTH, ROBOT_HALF_WIDTH)
ROBOT_LINEAR_LIMIT = 0.8
ROBOT_ANGULAR_LIMIT = 1.5
NUMERIC_ALLOWANCE = 0.005
MAX_SAMPLING_ALIGNMENT_ERROR = 0.06
TIMING = re.compile(r"solve_ms=([0-9.]+) cycle_ms=([0-9.]+)")


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def yaw_of(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


def footprint_circle_clearance(robot, human):
    x, y, yaw = robot
    dx = human[1] - x
    dy = human[2] - y
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    outside_x = max(abs(local_x) - ROBOT_HALF_LENGTH, 0.0)
    outside_y = max(abs(local_y) - ROBOT_HALF_WIDTH, 0.0)
    if outside_x > 0.0 or outside_y > 0.0:
        point_clearance = math.hypot(outside_x, outside_y)
    else:
        point_clearance = -min(
            ROBOT_HALF_LENGTH - abs(local_x), ROBOT_HALF_WIDTH - abs(local_y)
        )
    return point_clearance - human[3]


class HumanProbe(Node):
    def __init__(self, args):
        super().__init__(f"arena_mpc_p4_{args.scenario}_probe")
        self.args = args
        self.navigation = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.lifecycle_clients = [
            self.create_client(GetState, f"/{name}/get_state")
            for name in ("bt_navigator", "planner_server", "controller_server")
        ]
        self.bt_parameters = self.create_client(
            GetParameters, "/bt_navigator/get_parameters"
        )
        self.controller_parameters = self.create_client(
            GetParameters, "/controller_server/get_parameters"
        )
        self.bt_default_server_timeout_ms = None
        self.progress_required_movement_radius_m = None
        self.progress_movement_time_allowance_s = None
        self.progress_checker_plugin = None
        self.progress_status_timeout_s = None
        self.odom_samples = []
        self.human_samples = []
        self.outputs = []
        self.controller_status = []
        self.solve_ms = []
        self.cycle_ms = []
        self.latest_odom = None
        self.latest_humans = None
        self.action_started_wall = None
        self.id_change_published = 0
        self.backlog_published = 0
        self.backlog_queue = []
        self.create_subscription(Odometry, "/odom", self.on_odom, 50)
        self.create_subscription(Agents, "/human_states", self.on_humans, 20)
        self.create_subscription(Twist, "/cmd_vel", self.on_output, 50)
        self.create_subscription(String, "/FollowPath/status", self.on_status, 50)
        if args.inject_id_change or args.backlog_duration > 0.0:
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.human_publisher = self.create_publisher(Agents, "/human_states", qos)
            self.create_timer(0.02, self.publish_injections)
        else:
            self.human_publisher = None

    def on_odom(self, message):
        self.latest_odom = message
        pose = message.pose.pose
        self.odom_samples.append(
            (
                stamp_ns(message.header.stamp),
                pose.position.x,
                pose.position.y,
                yaw_of(pose.orientation),
                message.twist.twist.linear.x,
                message.twist.twist.angular.z,
            )
        )

    def on_humans(self, message):
        self.latest_humans = message
        agents = []
        for agent in message.agents:
            agents.append(
                (
                    agent.id,
                    agent.position.position.x,
                    agent.position.position.y,
                    agent.radius,
                    agent.velocity.linear.x,
                    agent.velocity.linear.y,
                    agent.desired_velocity,
                )
            )
        self.human_samples.append((stamp_ns(message.header.stamp), agents))
        if self.human_publisher is None or self.action_started_wall is None:
            return
        if any(agent.id >= 10_000 for agent in message.agents):
            return
        elapsed = time.monotonic() - self.action_started_wall
        if self.args.inject_id_change and self.id_change_published == 0 and elapsed >= 2.0:
            changed = copy.deepcopy(message)
            for agent in changed.agents:
                agent.id += 10_000
            self.human_publisher.publish(changed)
            self.id_change_published += 1
        if 2.0 <= elapsed <= 2.0 + self.args.backlog_duration:
            delayed = copy.deepcopy(message)
            for agent in delayed.agents:
                agent.id += 20_000
            self.backlog_queue.append(
                (time.monotonic() + self.args.backlog_delay, delayed)
            )

    def publish_injections(self):
        if self.human_publisher is None:
            return
        now = time.monotonic()
        ready = []
        while self.backlog_queue and self.backlog_queue[0][0] <= now:
            _, message = self.backlog_queue.pop(0)
            ready.append(message)
        for message in ready:
            self.human_publisher.publish(message)
            self.backlog_published += 1

    def on_output(self, message):
        self.outputs.append(
            (time.monotonic(), message.linear.x, message.angular.z)
        )

    def on_status(self, message):
        self.controller_status.append((time.monotonic(), message.data))
        match = TIMING.search(message.data)
        if match:
            self.solve_ms.append(float(match.group(1)))
            self.cycle_ms.append(float(match.group(2)))

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return True
        return False

    def wait_navigation_active(self):
        for client in self.lifecycle_clients:
            if not client.wait_for_service(timeout_sec=60.0):
                raise RuntimeError(f"lifecycle service unavailable: {client.srv_name}")
        deadline = time.monotonic() + 60.0
        states = []
        while time.monotonic() < deadline:
            states = []
            for client in self.lifecycle_clients:
                future = client.call_async(GetState.Request())
                if not self.spin_until(future.done, 5.0):
                    raise RuntimeError(f"lifecycle service timed out: {client.srv_name}")
                states.append(future.result().current_state.id)
            if all(value == State.PRIMARY_STATE_ACTIVE for value in states):
                return
            self.spin_until(lambda: False, 0.2)
        raise RuntimeError(f"navigation lifecycle did not become active: {states}")

    def read_runtime_parameters(self):
        if not self.bt_parameters.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("bt_navigator parameter service unavailable")
        request = GetParameters.Request()
        request.names = ["default_server_timeout"]
        future = self.bt_parameters.call_async(request)
        if not self.spin_until(future.done, 5.0):
            raise RuntimeError("bt_navigator parameter request timed out")
        response = future.result()
        if response is None or len(response.values) != 1:
            raise RuntimeError("bt_navigator parameter response was invalid")
        self.bt_default_server_timeout_ms = response.values[0].integer_value
        if not self.controller_parameters.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("controller_server parameter service unavailable")
        request = GetParameters.Request()
        request.names = [
            "progress_checker.plugin",
            "progress_checker.required_movement_radius",
            "progress_checker.movement_time_allowance",
            "progress_checker.status_timeout",
        ]
        future = self.controller_parameters.call_async(request)
        if not self.spin_until(future.done, 5.0):
            raise RuntimeError("controller_server parameter request timed out")
        response = future.result()
        if response is None or len(response.values) != 4:
            raise RuntimeError("controller_server parameter response was invalid")
        self.progress_checker_plugin = response.values[0].string_value
        self.progress_required_movement_radius_m = response.values[1].double_value
        self.progress_movement_time_allowance_s = response.values[2].double_value
        self.progress_status_timeout_s = response.values[3].double_value

    @staticmethod
    def interpolate_odom(samples, target_ns):
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
        yaw_delta = wrap(second[3] - first[3])
        return (
            first[1] + ratio * (second[1] - first[1]),
            first[2] + ratio * (second[2] - first[2]),
            first[3] + ratio * yaw_delta,
            interval * 1.0e-9,
        )

    def safety_report(self, start_ns, end_ns):
        odom = sorted({sample[0]: sample for sample in self.odom_samples}.values())
        humans = sorted(
            (sample for sample in self.human_samples if start_ns <= sample[0] <= end_ns),
            key=lambda item: item[0],
        )
        clearances = []
        clearances_by_id = {}
        odom_gaps = []
        ids_seen = set()
        id_sets = []
        max_human_speed = 0.0
        closest = None
        velocity_headings = {}
        stopped_after_motion = set()
        for human_stamp, agents in humans:
            robot = self.interpolate_odom(odom, human_stamp)
            if robot is None:
                continue
            odom_gaps.append(robot[3])
            current_ids = tuple(sorted(agent[0] for agent in agents))
            id_sets.append(current_ids)
            for agent in agents:
                ids_seen.add(agent[0])
                speed = math.hypot(agent[4], agent[5])
                max_human_speed = max(max_human_speed, speed, agent[6])
                if speed >= 0.05:
                    velocity_headings.setdefault(agent[0], []).append(
                        math.atan2(agent[5], agent[4])
                    )
                elif velocity_headings.get(agent[0]):
                    stopped_after_motion.add(agent[0])
                clearance = footprint_circle_clearance(robot[:3], agent)
                clearances.append(clearance)
                clearances_by_id.setdefault(agent[0], []).append(clearance)
                if closest is None or clearance < closest[0]:
                    closest = (clearance, human_stamp, agent[0], robot[:3], agent[1:4])

        human_stamps = sorted(set(item[0] for item in humans))
        coverage_stamps = [start_ns] + human_stamps + [end_ns]
        human_gaps = [
            (second - first) * 1.0e-9
            for first, second in zip(coverage_stamps, coverage_stamps[1:])
            if second >= first
        ]
        max_human_gap = max(human_gaps, default=math.inf)
        max_odom_gap = max(odom_gaps, default=math.inf)
        robot_corner_speed = (
            ROBOT_LINEAR_LIMIT
            + ROBOT_CIRCUMSCRIBED_RADIUS * ROBOT_ANGULAR_LIMIT
        )
        relative_speed_bound = robot_corner_speed + max_human_speed
        error_bound = (
            0.5 * relative_speed_bound * max_human_gap
            + 0.5 * robot_corner_speed * max_odom_gap
            + NUMERIC_ALLOWANCE
        )
        measured = min(clearances, default=None)
        lower = measured - error_bound if measured is not None else None
        measured_by_id = {
            str(agent_id): min(values)
            for agent_id, values in sorted(clearances_by_id.items())
        }
        lower_by_id = {
            agent_id: value - error_bound
            for agent_id, value in measured_by_id.items()
        }

        maximum_turn = 0.0
        maximum_turn_by_id = {}
        maximum_excursion_by_id = {}
        for headings in velocity_headings.values():
            for first, second in zip(headings, headings[1:]):
                maximum_turn = max(maximum_turn, abs(wrap(second - first)))
        for agent_id, headings in velocity_headings.items():
            maximum_turn_by_id[agent_id] = max(
                (
                    abs(wrap(second - first))
                    for first, second in zip(headings, headings[1:])
                ),
                default=0.0,
            )
            if headings:
                unwrapped = [headings[0]]
                for first, second in zip(headings, headings[1:]):
                    unwrapped.append(unwrapped[-1] + wrap(second - first))
                maximum_excursion_by_id[agent_id] = max(
                    abs(value - unwrapped[0]) for value in unwrapped
                )
        id_change_count = sum(
            first != second for first, second in zip(id_sets, id_sets[1:])
        )
        return {
            "aligned_human_samples": len(humans),
            "clearance_samples": len(clearances),
            "minimum_measured_footprint_human_clearance_m": measured,
            "sampling_alignment_error_bound_m": error_bound,
            "minimum_clearance_lower_bound_m": lower,
            "minimum_measured_clearance_by_id_m": measured_by_id,
            "minimum_clearance_lower_bound_by_id_m": lower_by_id,
            "maximum_human_stamp_gap_s": max_human_gap,
            "maximum_odom_bracket_s": max_odom_gap,
            "human_speed_bound_mps": max_human_speed,
            "ids_seen": sorted(ids_seen),
            "id_set_change_count": id_change_count,
            "stopped_after_motion_ids": sorted(stopped_after_motion),
            "maximum_step_velocity_heading_change_rad": maximum_turn,
            "maximum_step_velocity_heading_change_by_id_rad": {
                str(agent_id): value
                for agent_id, value in sorted(maximum_turn_by_id.items())
            },
            "maximum_velocity_heading_excursion_by_id_rad": {
                str(agent_id): value
                for agent_id, value in sorted(maximum_excursion_by_id.items())
            },
            "closest_sample": None
            if closest is None
            else {
                "stamp_ns": closest[1],
                "human_id": closest[2],
                "robot": list(closest[3]),
                "human": list(closest[4]),
            },
        }

    def run(self):
        if not self.spin_until(
            lambda: self.latest_odom is not None
            and self.latest_humans is not None
            and len(self.latest_humans.agents) == self.args.expected_agents
            and len(self.human_samples) >= 10,
            120.0,
        ):
            raise RuntimeError("timed out waiting for odom and expected HuNav state")
        if not self.navigation.wait_for_server(timeout_sec=60.0):
            raise RuntimeError("navigate_to_pose server unavailable")
        self.wait_navigation_active()
        self.read_runtime_parameters()

        start_pose = self.latest_odom.pose.pose
        target_x = start_pose.position.x + self.args.goal_dx
        target_y = start_pose.position.y + self.args.goal_dy
        target_yaw = self.args.goal_yaw
        start_ns = stamp_ns(self.latest_odom.header.stamp)
        start_wall = time.monotonic()
        self.action_started_wall = start_wall
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = target_x
        goal.pose.pose.position.y = target_y
        goal.pose.pose.orientation.z = math.sin(target_yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(target_yaw * 0.5)
        sent = self.navigation.send_goal_async(goal)
        if not self.spin_until(sent.done, 30.0):
            raise RuntimeError("goal acceptance timed out")
        handle = sent.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("goal rejected")
        result = handle.get_result_async()
        if not self.spin_until(result.done, self.args.timeout):
            cancel = handle.cancel_goal_async()
            self.spin_until(cancel.done, 5.0)
            raise RuntimeError("navigation timed out")
        status = result.result().status
        settle_end = time.monotonic() + 0.35
        while time.monotonic() < settle_end:
            rclpy.spin_once(self, timeout_sec=0.02)
        end_ns = stamp_ns(self.latest_odom.header.stamp)
        end_pose = self.latest_odom.pose.pose
        position_error = math.hypot(
            end_pose.position.x - target_x, end_pose.position.y - target_y
        )
        yaw_error = abs(wrap(yaw_of(end_pose.orientation) - target_yaw))
        safety = self.safety_report(start_ns, end_ns)
        reasons = Counter(
            text.split(" reason=", 1)[1].split(" solve_ms=", 1)[0]
            for _, text in self.controller_status
            if text.startswith("stop failure=") and " reason=" in text
        )
        recoverable_stops = Counter(
            text.split(" reason=", 1)[1]
            for _, text in self.controller_status
            if text.startswith("stop recoverable=1 ") and " reason=" in text
        )
        modes = Counter(
            match.group(1)
            for _, text in self.controller_status
            if text.startswith("ok ")
            for match in [re.search(r"(?:^| )mode=([^ ]+)", text)]
            if match
        )
        wait_reasons = Counter(
            match.group(1)
            for _, text in self.controller_status
            if text.startswith("ok ")
            for match in [re.search(r"(?:^| )wait_reason=([^ ]+)", text)]
            if match
        )
        outputs_finite = all(
            math.isfinite(linear) and math.isfinite(angular)
            for _, linear, angular in self.outputs
        )
        lower = safety["minimum_clearance_lower_bound_m"]
        measured = safety["minimum_measured_footprint_human_clearance_m"]
        error_bound = safety["sampling_alignment_error_bound_m"]
        turn_by_id = safety["maximum_velocity_heading_excursion_by_id_rad"]
        measured_by_id = safety["minimum_measured_clearance_by_id_m"]
        passed = (
            status == GoalStatus.STATUS_SUCCEEDED
            and position_error <= 0.25
            and yaw_error <= 0.25
            and (end_ns - start_ns) * 1.0e-9 <= 150.0
            and measured is not None
            and measured <= self.args.interaction_distance
            and error_bound <= MAX_SAMPLING_ALIGNMENT_ERROR
            and lower is not None
            and lower >= 0.30
            and outputs_finite
            and self.bt_default_server_timeout_ms == 500
            and self.progress_checker_plugin
            == "arena_mpc_controller::SafetyAwareProgressChecker"
            and abs(self.progress_required_movement_radius_m - 0.05) <= 1.0e-9
            and abs(self.progress_movement_time_allowance_s - 120.0) <= 1.0e-9
            and abs(self.progress_status_timeout_s - 1.0) <= 1.0e-9
            and len(safety["ids_seen"]) >= self.args.expected_agents
            and all(
                measured_by_id.get(str(agent_id), math.inf)
                <= self.args.interaction_distance
                for agent_id in self.args.require_interaction_agent
            )
            and (
                not self.args.inject_id_change
                or self.id_change_published >= 1
                and safety["id_set_change_count"] >= 1
            )
            and (
                self.args.backlog_duration <= 0.0 or self.backlog_published >= 1
            )
            and (
                self.args.require_stop_agent < 0
                or self.args.require_stop_agent in safety["stopped_after_motion_ids"]
            )
            and (
                self.args.require_turn_agent < 0
                or turn_by_id.get(str(self.args.require_turn_agent), 0.0)
                >= self.args.turn_threshold
            )
        )
        return {
            "scenario": self.args.scenario,
            "run_metadata": {
                "ros_domain_id": self.args.ros_domain_id,
                "gpu_index": self.args.gpu_index,
                "gpu_snapshot_csv": self.args.gpu_snapshot,
                "config_path": self.args.config_path,
                "config_sha256": self.args.config_sha256,
                "controller_source_sha256": self.args.controller_sha256,
                "probe_sha256": self.args.probe_sha256,
                "launch_sha256": self.args.launch_sha256,
                "nav2_overrides_sha256": self.args.nav2_overrides_sha256,
                "controller_config_sha256": self.args.controller_config_sha256,
                "git_head": self.args.git_head,
                "launch_log": self.args.launch_log,
                "webrtc_signal_port": self.args.webrtc_signal_port,
                "webrtc_media_port": self.args.webrtc_media_port,
                "foxglove_port": self.args.foxglove_port,
            },
            "action_status": status,
            "target": [target_x, target_y, target_yaw],
            "end": [
                end_pose.position.x,
                end_pose.position.y,
                yaw_of(end_pose.orientation),
            ],
            "position_error_m": position_error,
            "yaw_error_rad": yaw_error,
            "simulation_duration_s": (end_ns - start_ns) * 1.0e-9,
            "wall_duration_s": time.monotonic() - start_wall,
            "expected_agents": self.args.expected_agents,
            "interaction_distance_gate_m": self.args.interaction_distance,
            "outputs_finite": outputs_finite,
            "bt_default_server_timeout_ms": self.bt_default_server_timeout_ms,
            "progress_checker_plugin": self.progress_checker_plugin,
            "progress_required_movement_radius_m": (
                self.progress_required_movement_radius_m
            ),
            "progress_movement_time_allowance_s": (
                self.progress_movement_time_allowance_s
            ),
            "progress_status_timeout_s": self.progress_status_timeout_s,
            "controller_failure_counts": dict(reasons),
            "controller_recoverable_stop_counts": dict(recoverable_stops),
            "controller_mode_counts": dict(modes),
            "controller_wait_reason_counts": dict(wait_reasons),
            "id_change_injections": self.id_change_published,
            "backlog_injections": self.backlog_published,
            "backlog_delay_s": self.args.backlog_delay,
            "solver_samples": len(self.solve_ms),
            "solver_ms_p50": statistics.median(self.solve_ms) if self.solve_ms else None,
            "solver_ms_p95": percentile(self.solve_ms, 0.95),
            "solver_ms_max": max(self.solve_ms, default=None),
            "cycle_ms_p95": percentile(self.cycle_ms, 0.95),
            "cycle_ms_max": max(self.cycle_ms, default=None),
            "safety": safety,
            "pass": passed,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario")
    parser.add_argument("--goal-dx", type=float, required=True)
    parser.add_argument("--goal-dy", type=float, required=True)
    parser.add_argument("--goal-yaw", type=float, default=0.0)
    parser.add_argument("--expected-agents", type=int, required=True)
    parser.add_argument("--interaction-distance", type=float, default=1.5)
    parser.add_argument("--timeout", type=float, default=240.0)
    parser.add_argument("--inject-id-change", action="store_true")
    parser.add_argument("--backlog-duration", type=float, default=0.0)
    parser.add_argument("--backlog-delay", type=float, default=0.25)
    parser.add_argument("--require-stop-agent", type=int, default=-1)
    parser.add_argument("--require-turn-agent", type=int, default=-1)
    parser.add_argument("--require-interaction-agent", action="append", type=int, default=[])
    parser.add_argument("--turn-threshold", type=float, default=0.3)
    parser.add_argument("--output")
    parser.add_argument("--ros-domain-id", type=int, required=True)
    parser.add_argument("--gpu-index", type=int, required=True)
    parser.add_argument("--gpu-snapshot", required=True)
    parser.add_argument("--config-path", required=True)
    parser.add_argument("--config-sha256", required=True)
    parser.add_argument("--controller-sha256", required=True)
    parser.add_argument("--probe-sha256", required=True)
    parser.add_argument("--launch-sha256", required=True)
    parser.add_argument("--nav2-overrides-sha256", required=True)
    parser.add_argument("--controller-config-sha256", required=True)
    parser.add_argument("--git-head", required=True)
    parser.add_argument("--launch-log", required=True)
    parser.add_argument("--webrtc-signal-port", type=int, required=True)
    parser.add_argument("--webrtc-media-port", type=int, required=True)
    parser.add_argument("--foxglove-port", type=int, required=True)
    args = parser.parse_args()
    rclpy.init()
    probe = HumanProbe(args)
    exit_code = 0
    try:
        report = probe.run()
        if not report["pass"]:
            exit_code = 2
    except Exception as error:
        report = {"scenario": args.scenario, "error": str(error), "pass": False}
        exit_code = 1
    rendered = json.dumps(report, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered + "\n", encoding="utf-8")
    probe.destroy_node()
    rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
