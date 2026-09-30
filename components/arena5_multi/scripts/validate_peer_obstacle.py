#!/usr/bin/env python3
"""Validate peer occupancy, self exclusion, and old-position clearing."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/arena_multi_control"))
from arena_multi_control.scenario import load_scenario  # noqa: E402


class PeerProbe(Node):
    def __init__(self, observer: str, peer: str) -> None:
        super().__init__("multirobot_peer_obstacle_probe")
        self.poses: dict[str, tuple[float, float]] = {}
        self.costmap: OccupancyGrid | None = None
        self._probe_subscriptions = [
            self.create_subscription(
                Odometry, f"/{name}/odom",
                lambda message, robot=name: self.poses.__setitem__(
                    robot,
                    (float(message.pose.pose.position.x), float(message.pose.pose.position.y)),
                ),
                qos_profile_sensor_data,
            )
            for name in (observer, peer)
        ]
        costmap_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._probe_subscriptions.append(self.create_subscription(
            OccupancyGrid,
            f"/{observer}/local_costmap/costmap",
            self._on_costmap,
            costmap_qos,
        ))
        self.action_client = ActionClient(
            self, NavigateToPose, f"/{peer}/navigate_to_pose"
        )

    def _on_costmap(self, message: OccupancyGrid) -> None:
        self.costmap = message

    def cost_at(self, position: tuple[float, float]) -> int | None:
        if self.costmap is None:
            return None
        info = self.costmap.info
        mx = math.floor((position[0] - info.origin.position.x) / info.resolution)
        my = math.floor((position[1] - info.origin.position.y) / info.resolution)
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None
        return int(self.costmap.data[my * info.width + mx])

    def goal(self, x: float, y: float) -> NavigateToPose.Goal:
        result = NavigateToPose.Goal()
        result.pose = PoseStamped()
        result.pose.header.frame_id = "map"
        result.pose.header.stamp = self.get_clock().now().to_msg()
        result.pose.pose.position.x = x
        result.pose.pose.position.y = y
        result.pose.pose.orientation.w = 1.0
        return result


def spin_until(node: Node, predicate, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return bool(predicate())


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario")
    parser.add_argument("--move-distance", type=float, default=2.0)
    parser.add_argument("--goal-timeout", type=float, default=300.0)
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    scenario = load_scenario(args.scenario)
    robots = [robot for robot in scenario.robots if robot.control_mode == "nav2"]
    if len(robots) != 2:
        raise SystemExit("peer obstacle validation requires exactly two Nav2 robots")
    observer, peer = robots[0].name, robots[1].name

    rclpy.init()
    node = PeerProbe(observer, peer)
    failures: list[str] = []
    measurements: dict[str, object] = {}
    try:
        ready = spin_until(
            node,
            lambda: node.costmap is not None and observer in node.poses and peer in node.poses,
            60.0,
        )
        if not ready:
            failures.append("odometry or local costmap unavailable")
        elif not node.action_client.wait_for_server(timeout_sec=30.0):
            failures.append(f"{peer} NavigateToPose action unavailable")
        else:
            observer_pose = node.poses[observer]
            old_peer_pose = node.poses[peer]
            initial_peer_cost = node.cost_at(old_peer_pose)
            initial_self_cost = node.cost_at(observer_pose)
            measurements["initial"] = {
                "observer_pose": observer_pose,
                "peer_pose": old_peer_pose,
                "peer_center_cost": initial_peer_cost,
                "observer_center_cost": initial_self_cost,
            }
            if initial_peer_cost is None or initial_peer_cost < 99:
                failures.append(f"peer center was not lethal: {initial_peer_cost}")
            if initial_self_cost is None or initial_self_cost >= 99:
                failures.append(f"observer was not excluded from peer obstacles: {initial_self_cost}")

            target = (old_peer_pose[0] + args.move_distance, old_peer_pose[1])
            send_future = node.action_client.send_goal_async(node.goal(*target))
            if not spin_until(node, send_future.done, 30.0):
                failures.append("peer goal acceptance timeout")
            else:
                handle = send_future.result()
                if handle is None or not handle.accepted:
                    failures.append("peer move goal rejected")
                else:
                    result_future = handle.get_result_async()
                    if not spin_until(node, result_future.done, args.goal_timeout):
                        handle.cancel_goal_async()
                        failures.append("peer move goal timed out")
                    elif result_future.result().status != GoalStatus.STATUS_SUCCEEDED:
                        failures.append(
                            f"peer move status={result_future.result().status}"
                        )
                    else:
                        # Require both physical separation and a later costmap publication.
                        prior_stamp = node.costmap.header.stamp if node.costmap else None
                        moved = spin_until(
                            node,
                            lambda: math.hypot(
                                node.poses[peer][0] - old_peer_pose[0],
                                node.poses[peer][1] - old_peer_pose[1],
                            ) >= args.move_distance - 0.30
                            and node.costmap is not None
                            and (
                                prior_stamp is None
                                or node.costmap.header.stamp.sec != prior_stamp.sec
                                or node.costmap.header.stamp.nanosec != prior_stamp.nanosec
                            ),
                            10.0,
                        )
                        new_peer_pose = node.poses[peer]
                        old_cost = node.cost_at(old_peer_pose)
                        new_cost = node.cost_at(new_peer_pose)
                        self_cost = node.cost_at(node.poses[observer])
                        measurements["after_move"] = {
                            "peer_pose": new_peer_pose,
                            "peer_displacement_m": math.hypot(
                                new_peer_pose[0] - old_peer_pose[0],
                                new_peer_pose[1] - old_peer_pose[1],
                            ),
                            "old_peer_center_cost": old_cost,
                            "new_peer_center_cost": new_cost,
                            "observer_center_cost": self_cost,
                        }
                        if not moved:
                            failures.append("peer did not move far enough with a fresh costmap")
                        if new_cost is None or new_cost < 99:
                            failures.append(f"new peer center was not lethal: {new_cost}")
                        if old_cost is None or old_cost >= 99:
                            failures.append(f"old peer center was not cleared: {old_cost}")
                        if self_cost is None or self_cost >= 99:
                            failures.append(f"observer self exclusion was lost: {self_cost}")
    finally:
        report = {
            "passed": not failures,
            "observer": observer,
            "peer": peer,
            "ideal_perception": True,
            "measurements": measurements,
            "failures": failures,
        }
        output = Path(args.output) if args.output else Path(
            os.environ.get("ARENA_MULTI_RUN_DIR", str(ROOT / "evidence/runs/current"))
        ) / "peer_obstacle_validation.json"
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
