#!/usr/bin/env python3
import argparse
import json
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from hunav_msgs.msg import Agents
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class WatchdogLidarProbe(Node):
    def __init__(self):
        super().__init__("arena_mpc_p2_watchdog_lidar_probe")
        volatile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(String, "/p2/status", volatile)
        self.raw_pub = self.create_publisher(Twist, "/p2/raw", volatile)
        self.smooth_pub = self.create_publisher(Twist, "/p2/smooth", volatile)
        self.human_pub = self.create_publisher(Agents, "/p2/humans", volatile)
        self.odom_pub = self.create_publisher(Odometry, "/p2/odom", volatile)
        self.lidar_pub = self.create_publisher(LaserScan, "/p2/lidar", volatile)
        self.costmap_pub = self.create_publisher(Costmap, "/p2/costmap", transient)
        self.output_events = []
        self.watchdog_events = []
        self.lidar_enabled = True
        self.tick_count = 0
        self.create_subscription(Twist, "/p2/output", self.on_output, volatile)
        self.create_subscription(
            String, "/p2_watchdog/status", self.on_watchdog, volatile
        )
        self.create_timer(0.02, self.publish_inputs)

    def on_output(self, message):
        self.output_events.append(
            (time.monotonic(), message.linear.x, message.angular.z)
        )

    def on_watchdog(self, message):
        self.watchdog_events.append((time.monotonic(), message.data))

    def publish_inputs(self):
        now = self.get_clock().now().to_msg()
        smooth = Twist()
        smooth.linear.x = 0.1
        self.smooth_pub.publish(smooth)
        if self.tick_count % 5 == 0:
            status = String()
            status.data = "ok synthetic_cycle"
            self.status_pub.publish(status)
            raw = Twist()
            raw.linear.x = 0.1
            self.raw_pub.publish(raw)
            humans = Agents()
            humans.header.stamp = now
            humans.header.frame_id = "map"
            self.human_pub.publish(humans)
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "odom"
            self.odom_pub.publish(odom)
            if self.lidar_enabled:
                lidar = LaserScan()
                lidar.header.stamp = now
                lidar.header.frame_id = "lidar_link"
                self.lidar_pub.publish(lidar)
            costmap = Costmap()
            costmap.header.stamp = now
            costmap.header.frame_id = "odom"
            self.costmap_pub.publish(costmap)
        self.tick_count += 1

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return True
        return False

    @staticmethod
    def moving(event):
        return abs(event[1]) >= 0.05 or abs(event[2]) >= 0.05

    @staticmethod
    def stopped(event):
        return abs(event[1]) < 0.001 and abs(event[2]) < 0.001

    def run(self):
        if not self.spin_until(
            lambda: bool(self.output_events) and self.moving(self.output_events[-1]), 5.0
        ):
            raise RuntimeError("watchdog never forwarded the complete synthetic chain")
        cutoff = time.monotonic()
        self.lidar_enabled = False
        first_zero = None
        reason_time = None

        def lidar_stop_seen():
            nonlocal first_zero, reason_time
            for event in self.output_events:
                if event[0] >= cutoff and self.stopped(event):
                    first_zero = event[0]
                    break
            for stamp, text in self.watchdog_events:
                if stamp >= cutoff and text == "stop reason=lidar_input":
                    reason_time = stamp
                    break
            return first_zero is not None and reason_time is not None

        if not self.spin_until(lidar_stop_seen, 2.0):
            raise RuntimeError("lidar cutoff did not produce lidar_input stop")
        hold_end = time.monotonic() + 0.3
        while rclpy.ok() and time.monotonic() < hold_end:
            rclpy.spin_once(self, timeout_sec=0.02)
        rebound = sum(
            1
            for event in self.output_events
            if first_zero <= event[0] <= hold_end and self.moving(event)
        )
        return {
            "mode": "standalone_watchdog_lidar_cutoff",
            "first_zero_from_cutoff_s": first_zero - cutoff,
            "reason_from_cutoff_s": reason_time - cutoff,
            "nonzero_after_first_zero": rebound,
            "ros_age_timeout_s": 0.3,
            "pass": first_zero - cutoff <= 0.35 and rebound == 0,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output")
    args = parser.parse_args()
    rclpy.init()
    probe = WatchdogLidarProbe()
    exit_code = 0
    try:
        report = probe.run()
        if not report["pass"]:
            exit_code = 2
    except Exception as error:
        report = {
            "mode": "standalone_watchdog_lidar_cutoff",
            "error": str(error),
            "pass": False,
        }
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
