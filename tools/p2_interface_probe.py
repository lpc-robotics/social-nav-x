#!/usr/bin/env python3
import argparse
import json
import math
import re
import time
from pathlib import Path as FilePath

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped, Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath, NavigateToPose
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


GENERATION = re.compile(r"generation=([0-9]+)")


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quaternion_from_yaw(yaw):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


class InterfaceProbe(Node):
    def __init__(self, timeout):
        super().__init__("arena_mpc_p2_interface_probe")
        self.timeout = timeout
        self.navigation = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.follow_path = ActionClient(self, FollowPath, "/follow_path")
        self.speed_limit = self.create_publisher(SpeedLimit, "/speed_limit", 10)
        self.controller_change_state = self.create_client(
            ChangeState, "/controller_server/change_state"
        )
        self.costmap_change_state = self.create_client(
            ChangeState, "/local_costmap/local_costmap/change_state"
        )
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.odom = None
        self.output = None
        self.raw_events = []
        self.output_events = []
        self.status_events = []
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Twist, "/cmd_vel_nav", self.on_raw, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_output, 10)
        self.create_subscription(String, "/FollowPath/status", self.on_status, 10)

    def on_odom(self, message):
        self.odom = message

    def on_raw(self, message):
        self.raw_events.append(
            (time.monotonic(), message.linear.x, message.angular.z)
        )

    def on_output(self, message):
        self.output = message
        self.output_events.append(
            (time.monotonic(), message.linear.x, message.angular.z)
        )

    def on_status(self, message):
        self.status_events.append((time.monotonic(), message.data))

    def spin_until(self, predicate, timeout=None):
        deadline = time.monotonic() + (timeout or self.timeout)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return True
        return False

    def wait_ready(self):
        if not self.spin_until(lambda: self.odom is not None and self.output is not None, 30.0):
            raise RuntimeError("timed out waiting for odom and /cmd_vel")
        if not self.navigation.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("navigate_to_pose server unavailable")

    def current_pose(self):
        pose = self.odom.pose.pose
        return pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)

    def navigate(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation = quaternion_from_yaw(yaw)
        future = self.navigation.send_goal_async(goal)
        if not self.spin_until(future.done, 30.0):
            raise RuntimeError("goal acceptance timed out")
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("goal rejected")
        return handle, handle.get_result_async()

    def publish_speed_limit(self, value):
        message = SpeedLimit()
        message.percentage = False
        message.speed_limit = value
        for _ in range(5):
            self.speed_limit.publish(message)
            rclpy.spin_once(self, timeout_sec=0.05)

    def deactivate(self, client, name):
        if not client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError(f"{name} lifecycle service unavailable")
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_DEACTIVATE
        future = client.call_async(request)
        if not self.spin_until(future.done, 15.0):
            raise RuntimeError(f"{name} deactivate timed out")
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(f"{name} deactivate failed: {response}")

    def lifecycle_stop(self, target):
        self.wait_ready()
        start_x, start_y, start_yaw = self.current_pose()
        handle, result = self.navigate(
            start_x + math.cos(start_yaw),
            start_y + math.sin(start_yaw),
            start_yaw,
        )
        if not self.spin_until(
            lambda: self.output is not None and abs(self.output.linear.x) >= 0.05,
            30.0,
        ):
            raise RuntimeError(f"moving command not observed before {target} deactivate")
        request_time = time.monotonic()
        if target == "controller":
            self.deactivate(self.controller_change_state, target)
        else:
            self.deactivate(self.costmap_change_state, target)
        ack_time = time.monotonic()
        first_zero = None

        def zero_seen():
            nonlocal first_zero
            for stamp, linear, angular in self.output_events:
                if stamp >= request_time and abs(linear) < 0.001 and abs(angular) < 0.001:
                    first_zero = stamp
                    return True
            return False

        if not self.spin_until(zero_seen, 2.0):
            raise RuntimeError(f"zero command not observed after {target} deactivate")
        hold_end = time.monotonic() + 0.3
        while rclpy.ok() and time.monotonic() < hold_end:
            rclpy.spin_once(self, timeout_sec=0.02)
        later_nonzero = sum(
            1
            for stamp, linear, angular in self.output_events
            if first_zero <= stamp <= hold_end
            and (abs(linear) >= 0.001 or abs(angular) >= 0.001)
        )
        self.spin_until(result.done, 2.0)
        action_status = (
            result.result().status if result.done() else GoalStatus.STATUS_UNKNOWN
        )
        latency = first_zero - request_time
        return {
            "mode": target,
            "action_status": action_status,
            "deactivate_ack_latency_s": ack_time - request_time,
            "first_zero_latency_s": latency,
            "nonzero_after_first_zero": later_nonzero,
            "deadline_s": 0.35 if target == "costmap" else 0.27,
            "pass": latency <= (0.35 if target == "costmap" else 0.27)
            and later_nonzero == 0,
        }

    def speed(self, limit):
        self.wait_ready()
        self.publish_speed_limit(limit)
        start_x, start_y, start_yaw = self.current_pose()
        event_start = time.monotonic()
        _, result = self.navigate(
            start_x + 0.6 * math.cos(start_yaw),
            start_y + 0.6 * math.sin(start_yaw),
            start_yaw,
        )
        if not self.spin_until(result.done):
            raise RuntimeError("speed-limit navigation timed out")
        raw = [abs(v) for stamp, v, _ in self.raw_events if stamp >= event_start]
        self.publish_speed_limit(0.0)
        status = result.result().status
        maximum = max(raw, default=math.inf)
        return {
            "mode": "speed",
            "action_status": status,
            "speed_limit_mps": limit,
            "raw_linear_max_mps": maximum,
            "pass": status == GoalStatus.STATUS_SUCCEEDED and maximum <= limit + 0.002,
        }

    def orientation(self, delta):
        self.wait_ready()
        start_x, start_y, start_yaw = self.current_pose()
        wanted = wrap_angle(start_yaw + delta)
        _, result = self.navigate(start_x, start_y, wanted)
        if not self.spin_until(result.done):
            raise RuntimeError("orientation navigation timed out")
        _, _, end_yaw = self.current_pose()
        error = abs(wrap_angle(end_yaw - wanted))
        status = result.result().status
        return {
            "mode": "orientation",
            "action_status": status,
            "requested_delta_rad": delta,
            "end_yaw_error_rad": error,
            "pass": status == GoalStatus.STATUS_SUCCEEDED and error <= 0.25,
        }

    def cancel(self):
        self.wait_ready()
        start_x, start_y, start_yaw = self.current_pose()
        handle, result = self.navigate(
            start_x + math.cos(start_yaw),
            start_y + math.sin(start_yaw),
            start_yaw,
        )
        if not self.spin_until(
            lambda: self.output is not None
            and (abs(self.output.linear.x) >= 0.05 or abs(self.output.angular.z) >= 0.05),
            30.0,
        ):
            raise RuntimeError("moving command not observed before cancel")
        cancel_request = time.monotonic()
        canceled = handle.cancel_goal_async()
        if not self.spin_until(canceled.done, 10.0):
            raise RuntimeError("cancel request timed out")
        first_zero = None

        def zero_seen():
            nonlocal first_zero
            for stamp, linear, angular in self.output_events:
                if stamp >= cancel_request and abs(linear) < 0.001 and abs(angular) < 0.001:
                    first_zero = stamp
                    return True
            return False

        if not self.spin_until(zero_seen, 2.0):
            raise RuntimeError("zero command not observed after cancel")
        if not self.spin_until(result.done, 10.0):
            raise RuntimeError("canceled action did not finish")
        hold_end = time.monotonic() + 0.3
        while rclpy.ok() and time.monotonic() < hold_end:
            rclpy.spin_once(self, timeout_sec=0.02)
        later_nonzero = sum(
            1
            for stamp, linear, angular in self.output_events
            if first_zero <= stamp <= hold_end
            and (abs(linear) >= 0.001 or abs(angular) >= 0.001)
        )
        latency = first_zero - cancel_request
        status = result.result().status
        return {
            "mode": "cancel",
            "action_status": status,
            "first_zero_latency_s": latency,
            "nonzero_after_first_zero": later_nonzero,
            "pass": status == GoalStatus.STATUS_CANCELED
            and latency <= 0.27
            and later_nonzero == 0,
        }

    def preempt(self):
        self.wait_ready()
        start_x, start_y, start_yaw = self.current_pose()
        first_handle, first_result = self.navigate(
            start_x + 1.0 * math.cos(start_yaw),
            start_y + 1.0 * math.sin(start_yaw),
            start_yaw,
        )
        if not self.spin_until(
            lambda: self.output is not None and abs(self.output.linear.x) >= 0.05,
            30.0,
        ):
            raise RuntimeError("moving command not observed before preemption")
        replace_time = time.monotonic()
        current_x, current_y, _ = self.current_pose()
        second_handle, second_result = self.navigate(
            current_x - 0.6 * math.sin(start_yaw),
            current_y + 0.6 * math.cos(start_yaw),
            wrap_angle(start_yaw + math.pi / 2.0),
        )
        if not second_handle.accepted:
            raise RuntimeError("replacement goal rejected")
        if not self.spin_until(second_result.done):
            raise RuntimeError("replacement goal timed out")
        self.spin_until(first_result.done, 5.0)
        generations = []
        for stamp, text in self.status_events:
            if stamp < replace_time:
                continue
            match = GENERATION.search(text)
            if match:
                generations.append(int(match.group(1)))
        second_status = second_result.result().status
        first_status = (
            first_result.result().status if first_result.done() else GoalStatus.STATUS_UNKNOWN
        )
        return {
            "mode": "preempt",
            "first_action_status": first_status,
            "replacement_action_status": second_status,
            "post_replace_generations": sorted(set(generations)),
            "pass": second_status == GoalStatus.STATUS_SUCCEEDED
            and bool(generations),
        }

    def transformed_follow_path(self):
        self.wait_ready()
        if not self.follow_path.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("follow_path server unavailable")
        start_x, start_y, start_yaw = self.current_pose()
        tx, ty, frame_yaw = 1.0, -0.7, 0.35
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "p2_shifted"
        transform.transform.translation.x = tx
        transform.transform.translation.y = ty
        transform.transform.rotation = quaternion_from_yaw(frame_yaw)
        self.static_broadcaster.sendTransform(transform)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)

        path = Path()
        path.header.frame_id = "p2_shifted"
        path.header.stamp = self.get_clock().now().to_msg()
        cosine = math.cos(frame_yaw)
        sine = math.sin(frame_yaw)
        target_x = start_x + 0.6 * math.cos(start_yaw)
        target_y = start_y + 0.6 * math.sin(start_yaw)
        from geometry_msgs.msg import PoseStamped

        for index in range(21):
            ratio = index / 20.0
            world_x = start_x + ratio * (target_x - start_x)
            world_y = start_y + ratio * (target_y - start_y)
            shifted_x = cosine * (world_x - tx) + sine * (world_y - ty)
            shifted_y = -sine * (world_x - tx) + cosine * (world_y - ty)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = shifted_x
            pose.pose.position.y = shifted_y
            pose.pose.orientation = quaternion_from_yaw(start_yaw - frame_yaw)
            path.poses.append(pose)

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "goal_checker"
        sent = self.follow_path.send_goal_async(goal)
        if not self.spin_until(sent.done, 10.0):
            raise RuntimeError("transformed path acceptance timed out")
        handle = sent.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("transformed path rejected")
        result = handle.get_result_async()
        if not self.spin_until(result.done):
            raise RuntimeError("transformed path timed out")
        end_x, end_y, _ = self.current_pose()
        remaining = math.hypot(target_x - end_x, target_y - end_y)
        status = result.result().status
        return {
            "mode": "transform",
            "action_status": status,
            "transform_translation": [tx, ty],
            "transform_yaw_rad": frame_yaw,
            "target_error_m": remaining,
            "pass": status == GoalStatus.STATUS_SUCCEEDED and remaining <= 0.25,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "mode",
        choices=(
            "speed",
            "orientation",
            "cancel",
            "preempt",
            "transform",
            "controller",
            "costmap",
        ),
    )
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--speed-limit", type=float, default=0.08)
    parser.add_argument("--yaw-delta", type=float, default=0.8)
    parser.add_argument("--output")
    args = parser.parse_args()
    rclpy.init()
    probe = InterfaceProbe(args.timeout)
    exit_code = 0
    try:
        if args.mode == "speed":
            report = probe.speed(args.speed_limit)
        elif args.mode == "orientation":
            report = probe.orientation(args.yaw_delta)
        elif args.mode == "cancel":
            report = probe.cancel()
        elif args.mode == "preempt":
            report = probe.preempt()
        elif args.mode in ("controller", "costmap"):
            report = probe.lifecycle_stop(args.mode)
        else:
            report = probe.transformed_follow_path()
        if not report["pass"]:
            exit_code = 2
    except Exception as error:
        report = {"mode": args.mode, "error": str(error), "pass": False}
        exit_code = 1
    rendered = json.dumps(report, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        output = FilePath(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered + "\n", encoding="utf-8")
    probe.destroy_node()
    rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
