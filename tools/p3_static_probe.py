#!/usr/bin/env python3
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from isaacsim_msgs.msg import Wall
from isaacsim_msgs.srv import DeletePrims, SpawnWalls
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger


ROBOT_HALF_X = 0.24
ROBOT_HALF_Y = 0.22


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def robot_polygon(x, y, yaw):
    result = []
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    for local_x, local_y in (
        (ROBOT_HALF_X, ROBOT_HALF_Y),
        (ROBOT_HALF_X, -ROBOT_HALF_Y),
        (-ROBOT_HALF_X, -ROBOT_HALF_Y),
        (-ROBOT_HALF_X, ROBOT_HALF_Y),
    ):
        result.append(
            (
                x + cosine * local_x - sine * local_y,
                y + sine * local_x + cosine * local_y,
            )
        )
    return result


def wall_polygon(spec):
    x1, y1, x2, y2, thickness = spec
    dx = x2 - x1
    dy = y2 - y1
    length = math.hypot(dx, dy)
    nx = -dy / length * thickness * 0.5
    ny = dx / length * thickness * 0.5
    return [
        (x1 + nx, y1 + ny),
        (x2 + nx, y2 + ny),
        (x2 - nx, y2 - ny),
        (x1 - nx, y1 - ny),
    ]


def orientation(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def on_segment(a, b, p):
    return (
        min(a[0], b[0]) - 1.0e-9 <= p[0] <= max(a[0], b[0]) + 1.0e-9
        and min(a[1], b[1]) - 1.0e-9 <= p[1] <= max(a[1], b[1]) + 1.0e-9
    )


def segments_intersect(a, b, c, d):
    ab_c = orientation(a, b, c)
    ab_d = orientation(a, b, d)
    cd_a = orientation(c, d, a)
    cd_b = orientation(c, d, b)
    if ab_c * ab_d < 0.0 and cd_a * cd_b < 0.0:
        return True
    return (
        (abs(ab_c) <= 1.0e-9 and on_segment(a, b, c))
        or (abs(ab_d) <= 1.0e-9 and on_segment(a, b, d))
        or (abs(cd_a) <= 1.0e-9 and on_segment(c, d, a))
        or (abs(cd_b) <= 1.0e-9 and on_segment(c, d, b))
    )


def point_segment_distance(point, a, b):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= 1.0e-18:
        return math.dist(point, a)
    amount = max(
        0.0,
        min(1.0, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length_squared),
    )
    closest = (a[0] + amount * dx, a[1] + amount * dy)
    return math.dist(point, closest)


def polygon_distance(first, second):
    first_edges = list(zip(first, first[1:] + first[:1]))
    second_edges = list(zip(second, second[1:] + second[:1]))
    if any(
        segments_intersect(a, b, c, d)
        for a, b in first_edges
        for c, d in second_edges
    ):
        return 0.0
    return min(
        [point_segment_distance(point, c, d) for point in first for c, d in second_edges]
        + [point_segment_distance(point, a, b) for point in second for a, b in first_edges]
    )


class StaticProbe(Node):
    def __init__(self, scenario, repetitions, wall_timeout):
        super().__init__(f"arena_mpc_p3_{scenario}_probe")
        self.scenario = scenario
        self.repetitions = repetitions
        self.wall_timeout = wall_timeout
        self.navigation = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.spawn_client = self.create_client(SpawnWalls, "/isaac/SpawnWalls")
        self.delete_client = self.create_client(DeletePrims, "/isaac/DeletePrims")
        self.pause_client = self.create_client(Trigger, "/isaac/PauseSimulation")
        self.unpause_client = self.create_client(Trigger, "/isaac/UnpauseSimulation")
        self.clear_global = self.create_client(
            ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap"
        )
        self.clear_local = self.create_client(
            ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap"
        )
        self.lifecycle_clients = [
            self.create_client(GetState, f"/{name}/get_state")
            for name in ("bt_navigator", "planner_server", "controller_server")
        ]
        self.odom = None
        self.clock_ns = None
        self.global_costmap = None
        self.outputs = []
        self.wall_specs = []
        self.wall_names = []
        self.observed_wall_indices = set()
        self.minimum_wall_clearance = math.inf
        self.collision_samples = 0
        self.odom_samples = 0
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.create_subscription(Clock, "/clock", self.on_clock, 20)
        self.create_subscription(Twist, "/cmd_vel", self.on_output, 20)
        self.create_subscription(
            Costmap, "/global_costmap/costmap_raw", self.on_costmap, 10
        )

    def on_clock(self, message):
        self.clock_ns = message.clock.sec * 1_000_000_000 + message.clock.nanosec

    def on_output(self, message):
        self.outputs.append((time.monotonic(), message.linear.x, message.angular.z))

    def on_costmap(self, message):
        self.global_costmap = message
        self.record_observed_walls()

    def on_odom(self, message):
        self.odom = message
        self.odom_samples += 1
        if not self.wall_specs:
            return
        pose = message.pose.pose
        footprint = robot_polygon(
            pose.position.x, pose.position.y, yaw_of(pose.orientation)
        )
        clearance = min(
            polygon_distance(footprint, wall_polygon(spec))
            for spec in self.wall_specs
        )
        self.minimum_wall_clearance = min(self.minimum_wall_clearance, clearance)
        if clearance <= 1.0e-5:
            self.collision_samples += 1

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return True
        return False

    def call(self, client, request, timeout=30.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"service unavailable: {client.srv_name}")
        future = client.call_async(request)
        if not self.spin_until(future.done, timeout):
            raise RuntimeError(f"service timed out: {client.srv_name}")
        if future.exception() is not None:
            raise RuntimeError(f"service failed: {client.srv_name}: {future.exception()}")
        return future.result()

    def clear_costmaps(self):
        for client in (self.clear_global, self.clear_local):
            response = self.call(client, ClearEntireCostmap.Request())
            if response is None:
                raise RuntimeError(f"empty clear response: {client.srv_name}")

    def wait_navigation_active(self):
        deadline = time.monotonic() + 60.0
        for client in self.lifecycle_clients:
            if not client.wait_for_service(timeout_sec=60.0):
                raise RuntimeError(f"lifecycle service unavailable: {client.srv_name}")
        while time.monotonic() < deadline:
            states = []
            for client in self.lifecycle_clients:
                response = self.call(client, GetState.Request(), timeout=5.0)
                states.append(response.current_state.id if response is not None else 0)
            if all(state == State.PRIMARY_STATE_ACTIVE for state in states):
                return
            self.spin_until(lambda: False, 0.2)
        raise RuntimeError(f"navigation lifecycle did not become active: {states}")

    def configure(self, start_x, start_y, start_yaw):
        scenario = self.scenario
        walls = []
        expected_success = True
        check_observation = True

        def point(local_x, local_y, yaw_delta=0.0):
            cosine = math.cos(start_yaw)
            sine = math.sin(start_yaw)
            return (
                start_x + cosine * local_x - sine * local_y,
                start_y + sine * local_x + cosine * local_y,
                start_yaw + yaw_delta,
            )

        def wall(local_x1, local_y1, local_x2, local_y2, thickness):
            first = point(local_x1, local_y1)
            second = point(local_x2, local_y2)
            return (first[0], first[1], second[0], second[1], thickness)

        if scenario == "straight":
            targets = [point(0.8 * (index + 1), 0.0) for index in range(self.repetitions)]
            check_observation = False
        elif scenario == "turn":
            cycle = [
                point(0.6, 0.6, math.pi / 2.0),
                point(0.0, 1.2, math.pi),
                point(-0.6, 0.6, 3.0 * math.pi / 2.0),
                point(0.0, 0.0, 2.0 * math.pi),
            ]
            targets = [cycle[index % len(cycle)] for index in range(self.repetitions)]
            check_observation = False
        elif scenario == "orientation":
            targets = [
                point(0.0, 0.0, math.pi / 2.0 * (index + 1))
                for index in range(self.repetitions)
            ]
            check_observation = False
        elif scenario == "wall_edge":
            walls = [wall(-0.3, 0.55, 0.8 * self.repetitions + 0.4, 0.55, 0.1)]
            targets = [point(0.8 * (index + 1), 0.0) for index in range(self.repetitions)]
        elif scenario == "narrow":
            walls = [
                wall(-0.4, -0.5, 0.8 * self.repetitions + 0.4, -0.5, 0.1),
                wall(-0.4, 0.5, 0.8 * self.repetitions + 0.4, 0.5, 0.1),
            ]
            targets = [point(0.8 * (index + 1), 0.0) for index in range(self.repetitions)]
        elif scenario == "corner":
            # Traverse both ends of one fixed L-shaped obstacle.  This isolates
            # 90-degree corner following with useful footprint clearance; tight
            # corridors and constrained rotation have their own P3 scenarios.
            walls = [
                wall(0.9, -1.0, 0.9, 0.9, 0.1),
                wall(0.9, 0.9, 2.3, 0.9, 0.1),
            ]
            targets = [
                point(0.2, 1.5, math.pi / 2.0),
                point(1.6, 1.5, 0.0),
                point(2.8, 1.5, 0.0),
                point(2.8, 0.2, -math.pi / 2.0),
                point(2.8, -0.8, -math.pi / 2.0),
            ]
            targets = targets[: self.repetitions]
        elif scenario == "small_obstacle":
            spacing = 2.0
            obstacle_offset = 1.2
            walls = [
                wall(
                    spacing * index + obstacle_offset,
                    -0.25 if index % 2 == 0 else -0.15,
                    spacing * index + obstacle_offset,
                    0.25 if index % 2 == 0 else 0.35,
                    0.2,
                )
                for index in range(self.repetitions)
            ]
            targets = [point(spacing * (index + 1), 0.0) for index in range(self.repetitions)]
            # Later obstacles are initially occluded by earlier ones.  The
            # run-level gate requires each wall to be observed as the robot
            # advances instead of requiring simultaneous pre-visibility.
            check_observation = False
        elif scenario == "rotation_sweep":
            walls = [
                wall(-0.5, -0.5, 0.5, -0.5, 0.1),
                wall(-0.5, 0.5, 0.5, 0.5, 0.1),
            ]
            targets = [
                point(0.0, 0.0, math.pi / 2.0 * (index + 1))
                for index in range(self.repetitions)
            ]
        elif scenario == "unknown":
            targets = [(-1.0, start_y + 0.05 * index, start_yaw) for index in range(self.repetitions)]
            expected_success = False
            check_observation = False
        elif scenario == "blocked":
            half = 0.65
            walls = [
                wall(-half, -half, half, -half, 0.1),
                wall(half, -half, half, half, 0.1),
                wall(half, half, -half, half, 0.1),
                wall(-half, half, -half, -half, 0.1),
            ]
            targets = [point(1.5, 0.05 * index) for index in range(self.repetitions)]
            expected_success = False
        else:
            raise RuntimeError(f"unsupported scenario: {scenario}")
        return walls, targets, expected_success, check_observation

    def spawn_walls(self):
        if not self.wall_specs:
            return
        self.call(self.pause_client, Trigger.Request())
        request = SpawnWalls.Request()
        for index, spec in enumerate(self.wall_specs):
            x1, y1, x2, y2, thickness = spec
            wall = Wall()
            wall.name = f"p3_{self.scenario}_{index}"
            wall.start.x = x1
            wall.start.y = y1
            wall.start.z = 0.0
            wall.end.x = x2
            wall.end.y = y2
            wall.end.z = 2.5
            wall.thickness = thickness
            request.walls.append(wall)
            self.wall_names.append(wall.name)
        response = self.call(self.spawn_client, request)
        if response is None or len(response.ret) != len(request.walls) or not all(response.ret):
            raise RuntimeError(f"failed to spawn all walls: {response}")
        self.call(self.unpause_client, Trigger.Request())

    def delete_walls(self):
        if not self.wall_names:
            return
        try:
            self.call(self.pause_client, Trigger.Request())
            request = DeletePrims.Request()
            request.names = list(self.wall_names)
            response = self.call(self.delete_client, request)
            if response is None or len(response.ret) != len(request.names) or not all(response.ret):
                raise RuntimeError(f"failed to delete all walls: {response}")
        finally:
            self.call(self.unpause_client, Trigger.Request())
            self.wall_names = []

    def cost_at(self, x, y):
        message = self.global_costmap
        if message is None:
            return -1
        info = message.metadata
        column = math.floor((x - info.origin.position.x) / info.resolution)
        row = math.floor((y - info.origin.position.y) / info.resolution)
        if column < 0 or row < 0 or column >= info.size_x or row >= info.size_y:
            return -1
        return message.data[row * info.size_x + column]

    def record_observed_walls(self):
        if self.global_costmap is None:
            return
        for wall_index, spec in enumerate(self.wall_specs):
            x1, y1, x2, y2, _ = spec
            for index in range(11):
                amount = index / 10.0
                x = x1 + amount * (x2 - x1)
                y = y1 + amount * (y2 - y1)
                if self.cost_at(x, y) >= 90:
                    self.observed_wall_indices.add(wall_index)
                    break

    def walls_observed(self):
        self.record_observed_walls()
        return len(self.observed_wall_indices) == len(self.wall_specs)

    def navigate(self, target, expected_success):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp.sec = self.clock_ns // 1_000_000_000
        goal.pose.header.stamp.nanosec = self.clock_ns % 1_000_000_000
        goal.pose.pose.position.x = target[0]
        goal.pose.pose.position.y = target[1]
        goal.pose.pose.orientation.z = math.sin(target[2] * 0.5)
        goal.pose.pose.orientation.w = math.cos(target[2] * 0.5)
        start_ros = self.clock_ns
        start_wall = time.monotonic()
        sent = self.navigation.send_goal_async(goal)
        if not self.spin_until(sent.done, 30.0):
            raise RuntimeError("goal acceptance timed out")
        handle = sent.result()
        if handle is None or not handle.accepted:
            return {
                "accepted": False,
                "status": None,
                "expected_success": expected_success,
                "pass": not expected_success,
            }
        result = handle.get_result_async()
        if not self.spin_until(result.done, self.wall_timeout):
            cancel = handle.cancel_goal_async()
            self.spin_until(cancel.done, 5.0)
            raise RuntimeError("navigation result timed out")
        status = result.result().status
        succeeded = status == GoalStatus.STATUS_SUCCEEDED
        if succeeded:
            # controller_server declares success before the 20 Hz velocity
            # smoother has necessarily emitted and applied its final zero.
            # Measure the stopped pose rather than a callback race at result time.
            settle_end = time.monotonic() + 0.35
            while time.monotonic() < settle_end:
                rclpy.spin_once(self, timeout_sec=0.02)
        end_ros = self.clock_ns
        end_pose = self.odom.pose.pose
        position_error = math.hypot(
            end_pose.position.x - target[0], end_pose.position.y - target[1]
        )
        yaw_error = abs(wrap(yaw_of(end_pose.orientation) - target[2]))
        passed = (
            succeeded == expected_success
            and (not succeeded or (position_error <= 0.25 and yaw_error <= 0.25))
            and (end_ros - start_ros) * 1.0e-9 <= 150.0
        )
        if not expected_success:
            quiet_start = time.monotonic()
            quiet_end = quiet_start + 0.3
            while time.monotonic() < quiet_end:
                rclpy.spin_once(self, timeout_sec=0.02)
            rebound = any(
                stamp >= quiet_start and (abs(linear) >= 0.001 or abs(angular) >= 0.001)
                for stamp, linear, angular in self.outputs
            )
            passed = passed and not rebound
        else:
            rebound = None
        return {
            "accepted": True,
            "status": status,
            "expected_success": expected_success,
            "target": list(target),
            "end": [
                end_pose.position.x,
                end_pose.position.y,
                yaw_of(end_pose.orientation),
            ],
            "position_error_m": position_error,
            "yaw_error_rad": yaw_error,
            "sim_duration_s": (end_ros - start_ros) * 1.0e-9,
            "wall_duration_s": time.monotonic() - start_wall,
            "nonzero_rebound_after_failure": rebound,
            "pass": passed,
        }

    def run(self):
        if not self.spin_until(
            lambda: self.odom is not None
            and self.clock_ns is not None
            and self.global_costmap is not None,
            90.0,
        ):
            raise RuntimeError("timed out waiting for odom, clock, and global costmap")
        if not self.navigation.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("navigate_to_pose server unavailable")
        self.wait_navigation_active()
        start_pose = self.odom.pose.pose
        walls, targets, expected_success, check_observation = self.configure(
            start_pose.position.x,
            start_pose.position.y,
            yaw_of(start_pose.orientation),
        )
        self.wall_specs = walls
        self.clear_costmaps()
        self.spawn_walls()
        observed = True
        if check_observation:
            observed = self.spin_until(self.walls_observed, 20.0)
        elif walls:
            # SpawnWalls pauses simulation.  Require a post-unpause lidar /
            # costmap observation before sending the first goal so the
            # controller freshness gate is tested with live input.
            observed = self.spin_until(
                lambda: (self.record_observed_walls() is None)
                and 0 in self.observed_wall_indices,
                20.0,
            )
        if not observed:
            raise RuntimeError("spawned walls were not marked in global costmap")
        results = []
        for repetition, target in enumerate(targets):
            result = self.navigate(target, expected_success)
            result["repetition"] = repetition + 1
            results.append(result)
        observed = observed and self.walls_observed()
        clearance = (
            self.minimum_wall_clearance
            if math.isfinite(self.minimum_wall_clearance)
            else None
        )
        passed = (
            observed
            and len(results) == self.repetitions
            and all(item["pass"] for item in results)
            and self.collision_samples == 0
        )
        return {
            "scenario": self.scenario,
            "repetitions": self.repetitions,
            "expected_success": expected_success,
            "walls": [list(item) for item in walls],
            "walls_observed_in_global_costmap": observed,
            "observed_wall_count": len(self.observed_wall_indices),
            "minimum_exact_footprint_wall_clearance_m": clearance,
            "collision_samples": self.collision_samples,
            "odom_samples": self.odom_samples,
            "results": results,
            "pass": passed,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "scenario",
        choices=(
            "straight",
            "turn",
            "orientation",
            "wall_edge",
            "narrow",
            "corner",
            "small_obstacle",
            "rotation_sweep",
            "unknown",
            "blocked",
        ),
    )
    parser.add_argument("--repetitions", type=int, default=5)
    parser.add_argument("--wall-timeout", type=float, default=180.0)
    parser.add_argument("--output")
    args = parser.parse_args()
    rclpy.init()
    probe = StaticProbe(args.scenario, args.repetitions, args.wall_timeout)
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
