"""Small command-line clients for namespaced Nav2 actions."""

from __future__ import annotations

import argparse
import math
import sys

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def pose(x: float, y: float, yaw: float) -> PoseStamped:
    message = PoseStamped()
    message.header.frame_id = "map"
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.orientation.w = math.cos(yaw / 2.0)
    return message


def _robot(value: str) -> str:
    value = value.strip("/")
    if not value.startswith("robot_"):
        raise argparse.ArgumentTypeError("robot must be named robot_N")
    return value


def goal_main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("robot", type=_robot)
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("yaw", type=float)
    values = parser.parse_args(args)
    rclpy.init()
    node = Node("multi_nav_goal_client")
    client = ActionClient(node, NavigateToPose, f"/{values.robot}/navigate_to_pose")
    if not client.wait_for_server(timeout_sec=20.0):
        raise SystemExit("navigate_to_pose action unavailable")
    goal = NavigateToPose.Goal()
    goal.pose = pose(values.x, values.y, values.yaw)
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    send = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send)
    handle = send.result()
    if handle is None or not handle.accepted:
        raise SystemExit("goal rejected")
    result = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result)
    status = result.result().status
    print(f"robot={values.robot} status={status}")
    node.destroy_node()
    rclpy.shutdown()
    if status != 4:
        sys.exit(1)


def waypoints_main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("robot", type=_robot)
    parser.add_argument("points", nargs="+", help="x,y,yaw")
    values = parser.parse_args(args)
    points = []
    for raw in values.points:
        fields = [float(item) for item in raw.split(",")]
        if len(fields) != 3:
            raise SystemExit(f"invalid waypoint: {raw}")
        points.append(pose(*fields))
    rclpy.init()
    node = Node("multi_nav_waypoint_client")
    client = ActionClient(node, FollowWaypoints, f"/{values.robot}/follow_waypoints")
    if not client.wait_for_server(timeout_sec=20.0):
        raise SystemExit("follow_waypoints action unavailable")
    goal = FollowWaypoints.Goal()
    goal.poses = points
    send = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send)
    handle = send.result()
    if handle is None or not handle.accepted:
        raise SystemExit("waypoint goal rejected")
    result = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result)
    response = result.result()
    print(f"robot={values.robot} status={response.status} missed={list(response.result.missed_waypoints)}")
    node.destroy_node()
    rclpy.shutdown()
    if response.status != 4 or response.result.missed_waypoints:
        sys.exit(1)


def cancel_main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("robot", type=_robot)
    values = parser.parse_args(args)
    rclpy.init()
    node = Node("multi_nav_cancel_client")
    client = node.create_client(CancelGoal, f"/{values.robot}/navigate_to_pose/_action/cancel_goal")
    if not client.wait_for_service(timeout_sec=20.0):
        raise SystemExit("cancel service unavailable")
    future = client.call_async(CancelGoal.Request())
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    print(f"robot={values.robot} return_code={response.return_code} goals_canceling={len(response.goals_canceling)}")
    node.destroy_node()
    rclpy.shutdown()
    if response.return_code != CancelGoal.Response.ERROR_NONE:
        sys.exit(1)

