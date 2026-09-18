#!/usr/bin/env python3
"""Thirty-minute MPC endurance and complete-command timing probe."""

import argparse
import bisect
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
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


TIMING = re.compile(r"solve_ms=([0-9.]+) cycle_ms=([0-9.]+)")


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


class EnduranceProbe(Node):
    def __init__(self, args):
        super().__init__("p5_mpc_endurance_probe")
        self.args = args
        self.navigation = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.lifecycle_clients = [
            self.create_client(GetState, f"/{name}/get_state")
            for name in ("bt_navigator", "planner_server", "controller_server")
        ]
        self.controller_parameters = self.create_client(
            GetParameters, "/controller_server/get_parameters"
        )
        self.progress_required_movement_radius_m = None
        self.progress_movement_time_allowance_s = None
        self.progress_checker_plugin = None
        self.progress_status_timeout_s = None
        self.latest_odom = None
        self.latest_humans = None
        self.latest_lidar = None
        self.latest_costmap = None
        self.last_arrival = {}
        self.maximum_gap = Counter()
        self.message_count = Counter()
        self.outputs = []
        self.status = []
        self.watchdog_status = []
        self.timed_cycles = []
        self.solve_ms = []
        self.raw_times = []
        self.clock_first_ns = None
        self.clock_last_ns = None
        self.clock_rollbacks = 0
        self.create_subscription(Odometry, "/odom", self.on_odom, 50)
        self.create_subscription(Agents, "/human_states", self.on_humans, 20)
        self.create_subscription(LaserScan, "/lidar", self.on_lidar, 20)
        costmap_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Costmap, "/local_costmap/costmap_raw", self.on_costmap, costmap_qos
        )
        self.create_subscription(Twist, "/cmd_vel_nav", self.on_raw, 50)
        self.create_subscription(Twist, "/cmd_vel", self.on_output, 50)
        self.create_subscription(String, "/FollowPath/status", self.on_status, 50)
        self.create_subscription(
            String, "/mpc_command_watchdog/status", self.on_watchdog_status, 20
        )

    def arrival(self, topic):
        now = time.monotonic()
        if topic in self.last_arrival:
            self.maximum_gap[topic] = max(
                self.maximum_gap[topic], now - self.last_arrival[topic]
            )
        self.last_arrival[topic] = now
        self.message_count[topic] += 1

    def observe_clock(self, stamp):
        value = stamp.sec * 1_000_000_000 + stamp.nanosec
        if self.clock_last_ns is not None and value < self.clock_last_ns:
            self.clock_rollbacks += 1
        if self.clock_first_ns is None:
            self.clock_first_ns = value
        self.clock_last_ns = value

    def on_odom(self, message):
        self.arrival("odom")
        self.observe_clock(message.header.stamp)
        self.latest_odom = message

    def on_humans(self, message):
        self.arrival("human_states")
        self.latest_humans = message

    def on_lidar(self, message):
        self.arrival("lidar")
        self.latest_lidar = message

    def on_costmap(self, message):
        self.arrival("local_costmap")
        self.latest_costmap = message

    def on_raw(self, _message):
        self.raw_times.append(time.monotonic())

    def on_output(self, message):
        self.outputs.append((time.monotonic(), message.linear.x, message.angular.z))

    def on_status(self, message):
        now = time.monotonic()
        self.status.append((now, message.data))
        match = TIMING.search(message.data)
        if match:
            solve = float(match.group(1))
            cycle = float(match.group(2))
            self.solve_ms.append(solve)
            self.timed_cycles.append((now, cycle, message.data.startswith("ok ")))

    def on_watchdog_status(self, message):
        self.watchdog_status.append((time.monotonic(), message.data))

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
                    raise RuntimeError(
                        f"lifecycle service timed out: {client.srv_name}"
                    )
                states.append(future.result().current_state.id)
            if all(value == State.PRIMARY_STATE_ACTIVE for value in states):
                return
            self.spin_until(lambda: False, 0.2)
        raise RuntimeError(f"navigation lifecycle did not become active: {states}")

    def make_goal(self, x):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.latest_odom.header.stamp
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = 3.0
        goal.pose.pose.orientation.w = 1.0
        return goal

    def read_progress_parameters(self):
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

    def run(self):
        ready = self.spin_until(
            lambda: self.latest_odom is not None
            and self.latest_humans is not None
            and len(self.latest_humans.agents) == 6
            and self.latest_lidar is not None
            and self.latest_costmap is not None,
            self.args.startup_timeout,
        )
        if not ready:
            missing = []
            if self.latest_odom is None:
                missing.append("odom")
            if self.latest_humans is None or len(self.latest_humans.agents) != 6:
                missing.append("six-agent human_states")
            if self.latest_lidar is None:
                missing.append("lidar")
            if self.latest_costmap is None:
                missing.append("local_costmap")
            raise RuntimeError(f"MPC runtime did not become ready: missing {missing}")
        if not self.navigation.wait_for_server(timeout_sec=60.0):
            raise RuntimeError("navigate_to_pose server unavailable")
        self.wait_navigation_active()
        self.read_progress_parameters()

        initial_stamp = self.latest_odom.header.stamp
        initial_ns = initial_stamp.sec * 1_000_000_000 + initial_stamp.nanosec
        self.last_arrival.clear()
        self.maximum_gap.clear()
        self.message_count.clear()
        self.outputs.clear()
        self.status.clear()
        self.watchdog_status.clear()
        self.timed_cycles.clear()
        self.solve_ms.clear()
        self.raw_times.clear()
        self.clock_first_ns = initial_ns
        self.clock_last_ns = initial_ns
        self.clock_rollbacks = 0
        start_wall = time.monotonic()
        end_wall = start_wall + self.args.duration
        completed = []
        goal_index = 0
        timed_out_goals = 0
        cancelled_at_end = False
        while time.monotonic() < end_wall:
            target_x = (3.6, 3.0)[goal_index % 2]
            sent = self.navigation.send_goal_async(self.make_goal(target_x))
            if not self.spin_until(sent.done, 30.0):
                raise RuntimeError("endurance goal acknowledgement timed out")
            handle = sent.result()
            if handle is None or not handle.accepted:
                raise RuntimeError("endurance goal was rejected")
            result = handle.get_result_async()
            action_deadline = min(end_wall, time.monotonic() + self.args.goal_timeout)
            while not result.done() and time.monotonic() < action_deadline:
                rclpy.spin_once(self, timeout_sec=0.02)
            if result.done():
                completed.append(
                    {
                        "target_x": target_x,
                        "status": result.result().status,
                        "elapsed_wall_s": time.monotonic() - start_wall,
                    }
                )
                goal_index += 1
                continue
            cancel = handle.cancel_goal_async()
            if not self.spin_until(cancel.done, 5.0):
                raise RuntimeError("endurance goal cancellation timed out")
            if time.monotonic() >= end_wall:
                cancelled_at_end = True
                break
            response = cancel.result()
            if response is None or not response.goals_canceling:
                raise RuntimeError("endurance goal cancellation was rejected")
            if not self.spin_until(result.done, 10.0):
                raise RuntimeError("cancelled endurance goal did not terminate")
            completed.append(
                {
                    "target_x": target_x,
                    "status": result.result().status,
                    "elapsed_wall_s": time.monotonic() - start_wall,
                    "timed_out": True,
                }
            )
            timed_out_goals += 1
            goal_index += 1

        self.spin_until(lambda: False, 0.5)
        wall_duration = time.monotonic() - start_wall
        # Status is published inside the plugin immediately before it returns;
        # controller_server then publishes the headerless raw Twist.  DDS does
        # not define delivery order across those two topics, so receipt-order
        # matching can incorrectly pair a status with the next 10 Hz command.
        # Match both orders within one quarter-period, then add the observed
        # boundary p99 to every directly measured plugin cycle as a conservative
        # server/publication bound.
        matching_window_s = 0.025
        ok_cycles = [item for item in self.timed_cycles if item[2]]
        candidates = []
        for status_index, (status_time, _, _) in enumerate(ok_cycles):
            insertion = bisect.bisect_left(self.raw_times, status_time)
            for raw_index in range(max(0, insertion - 2), min(len(self.raw_times), insertion + 2)):
                separation = abs(self.raw_times[raw_index] - status_time)
                if separation <= matching_window_s:
                    candidates.append((separation, status_index, raw_index))
        candidates.sort()
        matched_status = set()
        matched_raw = set()
        status_to_raw_ms = []
        for separation, status_index, raw_index in candidates:
            if status_index in matched_status or raw_index in matched_raw:
                continue
            matched_status.add(status_index)
            matched_raw.add(raw_index)
            status_to_raw_ms.append(1000.0 * separation)
        match_coverage = (
            len(matched_status) / len(ok_cycles) if ok_cycles else None
        )
        boundary_p99 = percentile(status_to_raw_ms, 0.99)
        full_command_ms = (
            [cycle_ms + boundary_p99 for _, cycle_ms, _ in self.timed_cycles]
            if boundary_p99 is not None
            else []
        )

        failure_reasons = Counter(
            text.split(" reason=", 1)[1].split(" solve_ms=", 1)[0]
            for _, text in self.status
            if text.startswith("stop failure=") and " reason=" in text
        )
        recoverable_stops = Counter(
            text.split(" reason=", 1)[1]
            for _, text in self.status
            if text.startswith("stop recoverable=1 ") and " reason=" in text
        )
        watchdog_reasons = Counter(
            text.removeprefix("stop reason=")
            for _, text in self.watchdog_status
            if text.startswith("stop reason=")
        )
        finite = all(
            math.isfinite(linear) and math.isfinite(angular)
            for _, linear, angular in self.outputs
        )
        output_publishers = sorted(
            f"{info.node_namespace.rstrip('/')}/{info.node_name}".replace("//", "/")
            for info in self.get_publishers_info_by_topic("/cmd_vel")
        )
        full_p95 = percentile(full_command_ms, 0.95)
        full_p99 = percentile(full_command_ms, 0.99)
        over_deadline = (
            sum(value > 100.0 for value in full_command_ms) / len(full_command_ms)
            if full_command_ms
            else None
        )
        success_count = sum(
            item["status"] == GoalStatus.STATUS_SUCCEEDED for item in completed
        )
        aborted_count = sum(
            item["status"] == GoalStatus.STATUS_ABORTED for item in completed
        )
        non_success_count = len(completed) - success_count
        stream_limits = {
            "human_states": 1.2,
            "odom": 1.2,
            "lidar": 3.1,
            "local_costmap": 3.1,
        }
        streams_healthy = all(
            self.maximum_gap[name] <= limit
            and self.message_count[name] >= 10
            for name, limit in stream_limits.items()
        )
        simulation_duration = (
            (self.clock_last_ns - self.clock_first_ns) * 1e-9
            if self.clock_first_ns is not None and self.clock_last_ns is not None
            else None
        )
        passed = (
            wall_duration >= self.args.duration
            and success_count >= self.args.minimum_goals
            and aborted_count == 0
            and timed_out_goals == 0
            and self.progress_checker_plugin
            == "arena_mpc_controller::SafetyAwareProgressChecker"
            and abs(self.progress_required_movement_radius_m - 0.05) <= 1.0e-9
            and abs(self.progress_movement_time_allowance_s - 120.0) <= 1.0e-9
            and abs(self.progress_status_timeout_s - 1.0) <= 1.0e-9
            and finite
            and len(self.outputs) > 0
            and full_p95 is not None
            and full_p95 <= 90.0
            and full_p99 <= 100.0
            and over_deadline <= 0.01
            and match_coverage is not None
            and match_coverage >= 0.95
            and boundary_p99 <= 10.0
            and streams_healthy
            and self.clock_rollbacks == 0
            and output_publishers == ["/mpc_command_watchdog"]
        )
        return {
            "pass": passed,
            "run_metadata": {
                "ros_domain_id": self.args.ros_domain_id,
                "gpu_index": self.args.gpu_index,
                "gpu_snapshot_csv": self.args.gpu_snapshot,
                "config_path": self.args.config_path,
                "config_sha256": self.args.config_sha256,
                "controller_source_sha256": self.args.controller_sha256,
                "progress_checker_source_sha256": self.args.progress_checker_sha256,
                "controller_config_sha256": self.args.controller_config_sha256,
                "nav2_overrides_sha256": self.args.nav2_overrides_sha256,
                "probe_sha256": self.args.probe_sha256,
                "git_head": self.args.git_head,
                "launch_log": self.args.launch_log,
            },
            "requested_wall_duration_s": self.args.duration,
            "observed_wall_duration_s": wall_duration,
            "simulation_duration_s": simulation_duration,
            "completed_goals": completed,
            "completed_goal_count": len(completed),
            "successful_goal_count": success_count,
            "aborted_goal_count": aborted_count,
            "non_successful_goal_count": non_success_count,
            "timed_out_goal_count": timed_out_goals,
            "progress_checker_plugin": self.progress_checker_plugin,
            "progress_required_movement_radius_m": (
                self.progress_required_movement_radius_m
            ),
            "progress_movement_time_allowance_s": (
                self.progress_movement_time_allowance_s
            ),
            "progress_status_timeout_s": self.progress_status_timeout_s,
            "cancelled_active_goal_at_end": cancelled_at_end,
            "outputs_finite": finite,
            "output_samples": len(self.outputs),
            "cmd_vel_publishers": output_publishers,
            "controller_failure_counts": dict(failure_reasons),
            "controller_recoverable_stop_counts": dict(recoverable_stops),
            "watchdog_stop_counts": dict(watchdog_reasons),
            "solver_samples": len(self.solve_ms),
            "solver_ms_p50": statistics.median(self.solve_ms) if self.solve_ms else None,
            "solver_ms_p95": percentile(self.solve_ms, 0.95),
            "solver_ms_p99": percentile(self.solve_ms, 0.99),
            "solver_ms_max": max(self.solve_ms, default=None),
            "plugin_cycle_ms_p50": statistics.median(
                [item[1] for item in self.timed_cycles]
            ) if self.timed_cycles else None,
            "plugin_cycle_ms_p95": percentile(
                [item[1] for item in self.timed_cycles], 0.95
            ),
            "plugin_cycle_ms_p99": percentile(
                [item[1] for item in self.timed_cycles], 0.99
            ),
            "plugin_cycle_ms_max": max(
                (item[1] for item in self.timed_cycles), default=None
            ),
            "full_command_measurement": (
                "plugin cycle plus p99 absolute callback skew from nearest "
                "same-cycle ok-status/raw pairs within 25 ms"
            ),
            "ok_status_samples": len(ok_cycles),
            "cross_boundary_samples": len(matched_status),
            "full_command_samples": len(full_command_ms),
            "matched_status_raw_samples": len(matched_status),
            "status_raw_match_coverage": match_coverage,
            "status_to_raw_ms_p95": percentile(status_to_raw_ms, 0.95),
            "status_to_raw_ms_p99": boundary_p99,
            "status_to_raw_ms_max": max(status_to_raw_ms, default=None),
            "full_command_ms_p95": full_p95,
            "full_command_ms_p99": full_p99,
            "full_command_ms_max": max(full_command_ms, default=None),
            "full_command_over_100ms_fraction": over_deadline,
            "stream_message_counts": dict(self.message_count),
            "stream_maximum_wall_gap_s": dict(self.maximum_gap),
            "stream_gap_limits_s": stream_limits,
            "streams_healthy": streams_healthy,
            "clock_rollbacks": self.clock_rollbacks,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1800.0)
    parser.add_argument("--startup-timeout", type=float, default=180.0)
    parser.add_argument("--goal-timeout", type=float, default=600.0)
    parser.add_argument("--minimum-goals", type=int, default=10)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ros-domain-id", type=int, required=True)
    parser.add_argument("--gpu-index", type=int, required=True)
    parser.add_argument("--gpu-snapshot", required=True)
    parser.add_argument("--config-path", required=True)
    parser.add_argument("--config-sha256", required=True)
    parser.add_argument("--controller-sha256", required=True)
    parser.add_argument("--progress-checker-sha256", required=True)
    parser.add_argument("--controller-config-sha256", required=True)
    parser.add_argument("--nav2-overrides-sha256", required=True)
    parser.add_argument("--probe-sha256", required=True)
    parser.add_argument("--git-head", required=True)
    parser.add_argument("--launch-log", required=True)
    args = parser.parse_args()
    rclpy.init()
    probe = EnduranceProbe(args)
    exit_code = 0
    try:
        report = probe.run()
        if not report["pass"]:
            exit_code = 2
    except Exception as error:
        report = {"pass": False, "error": str(error)}
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
