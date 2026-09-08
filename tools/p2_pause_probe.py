#!/usr/bin/env python3
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class PauseProbe(Node):
    def __init__(self, travel, wall_timeout):
        super().__init__("arena_mpc_p2_pause_probe")
        self.travel = travel
        self.wall_timeout = wall_timeout
        self.navigation = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.pause = self.create_client(Trigger, "/isaac/PauseSimulation")
        self.unpause = self.create_client(Trigger, "/isaac/UnpauseSimulation")
        self.odom = None
        self.last_odom_wall = None
        self.output = None
        self.output_events = []
        self.watchdog_events = []
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_output, 10)
        self.create_subscription(
            String, "/mpc_command_watchdog/status", self.on_watchdog, 10
        )

    def on_odom(self, message):
        self.odom = message
        self.last_odom_wall = time.monotonic()

    def on_output(self, message):
        self.output = message
        self.output_events.append(
            (time.monotonic(), message.linear.x, message.angular.z)
        )

    def on_watchdog(self, message):
        self.watchdog_events.append((time.monotonic(), message.data))

    def spin_until(self, predicate, timeout=None):
        deadline = time.monotonic() + (timeout or self.wall_timeout)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return True
        return False

    def call(self, client, name):
        if not client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError(f"{name} service unavailable")
        future = client.call_async(Trigger.Request())
        if not self.spin_until(future.done, 15.0):
            raise RuntimeError(f"{name} service timed out")
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(f"{name} service failed: {response}")
        return response.message

    @staticmethod
    def stopped(event):
        return abs(event[1]) < 0.001 and abs(event[2]) < 0.001

    def run(self):
        if not self.spin_until(lambda: self.odom is not None and self.output is not None, 30.0):
            raise RuntimeError("timed out waiting for odom and /cmd_vel")
        if not self.navigation.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("navigate_to_pose server unavailable")

        start_x = self.odom.pose.pose.position.x
        start_y = self.odom.pose.pose.position.y
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = start_x + self.travel
        goal.pose.pose.position.y = start_y
        goal.pose.pose.orientation.w = 1.0
        sent = self.navigation.send_goal_async(goal)
        if not self.spin_until(sent.done, 10.0):
            raise RuntimeError("goal acceptance timed out")
        handle = sent.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("goal rejected")
        result = handle.get_result_async()

        if not self.spin_until(
            lambda: self.output is not None and abs(self.output.linear.x) >= 0.05,
            30.0,
        ):
            raise RuntimeError("robot never received a moving command")
        pause_request = time.monotonic()
        last_odom_before_pause = self.last_odom_wall
        pause_message = self.call(self.pause, "pause")
        pause_ack = time.monotonic()

        first_zero = None
        stop_reason = None

        def observed_watchdog_stop():
            nonlocal first_zero, stop_reason
            for event in self.output_events:
                if event[0] >= pause_request and self.stopped(event):
                    first_zero = event[0]
                    break
            for stamp, text in self.watchdog_events:
                if stamp >= pause_request and text.startswith("stop reason="):
                    stop_reason = text
                    break
            return first_zero is not None and stop_reason is not None

        if not self.spin_until(observed_watchdog_stop, 2.0):
            raise RuntimeError("watchdog did not emit zero and a stop reason")

        cancel = handle.cancel_goal_async()
        if not self.spin_until(cancel.done, 10.0):
            raise RuntimeError("goal cancellation timed out while paused")
        cancel_ack = time.monotonic()

        hold_deadline = time.monotonic() + 0.35
        while rclpy.ok() and time.monotonic() < hold_deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        nonzero_after_first_zero = sum(
            1
            for event in self.output_events
            if first_zero <= event[0] <= hold_deadline and not self.stopped(event)
        )

        unpause_message = self.call(self.unpause, "unpause")
        unpause_ros = (
            self.odom.header.stamp.sec + self.odom.header.stamp.nanosec * 1.0e-9
        )
        stopped_ros = None

        def robot_stopped():
            nonlocal stopped_ros
            if self.odom is None:
                return False
            linear = math.hypot(
                self.odom.twist.twist.linear.x, self.odom.twist.twist.linear.y
            )
            angular = abs(self.odom.twist.twist.angular.z)
            now_ros = (
                self.odom.header.stamp.sec
                + self.odom.header.stamp.nanosec * 1.0e-9
            )
            if now_ros >= unpause_ros and linear < 0.02 and angular < 0.05:
                stopped_ros = now_ros
                return True
            return False

        if not self.spin_until(robot_stopped, 15.0):
            raise RuntimeError("robot did not stop after simulation resumed")
        self.spin_until(result.done, 5.0)
        action_status = result.result().status if result.done() else GoalStatus.STATUS_UNKNOWN

        input_deadline_pass = True
        first_zero_from_last_odom = None
        if stop_reason == "stop reason=odom_input":
            first_zero_from_last_odom = first_zero - last_odom_before_pause
            # Pause acknowledgement is the latest point at which Isaac may
            # still have emitted odometry.  Cross-process subscriber callback
            # times are not a valid proxy for the watchdog's own reception
            # time, so evaluate its 0.40 s lease plus one 20 ms tick from ACK.
            input_deadline_pass = first_zero - pause_ack <= 0.42

        passed = (
            input_deadline_pass
            and nonzero_after_first_zero == 0
            and stopped_ros - unpause_ros <= 0.6
        )
        return {
            "action_status": action_status,
            "pause_service_message": pause_message,
            "unpause_service_message": unpause_message,
            "pause_ack_latency_s": pause_ack - pause_request,
            "first_zero_from_pause_request_s": first_zero - pause_request,
            "first_zero_from_pause_ack_s": first_zero - pause_ack,
            "first_zero_from_last_odom_s": first_zero_from_last_odom,
            "watchdog_stop_reason": stop_reason,
            "nonzero_commands_after_first_zero_while_paused": nonzero_after_first_zero,
            "cancel_ack_from_pause_request_s": cancel_ack - pause_request,
            "robot_stop_sim_time_after_unpause_s": stopped_ros - unpause_ros,
            "pass_watchdog_deadline": input_deadline_pass,
            "pass_continuous_zero": nonzero_after_first_zero == 0,
            "pass_robot_stop": stopped_ros - unpause_ros <= 0.6,
            "pass": passed,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--travel", type=float, default=2.0)
    parser.add_argument("--wall-timeout", type=float, default=60.0)
    parser.add_argument("--output")
    args = parser.parse_args()
    rclpy.init()
    probe = PauseProbe(args.travel, args.wall_timeout)
    exit_code = 0
    try:
        report = probe.run()
        if not report["pass"]:
            exit_code = 2
    except Exception as error:
        report = {"error": str(error)}
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
