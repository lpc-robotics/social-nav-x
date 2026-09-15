#!/usr/bin/env python3
"""Publish MPC and HuNav data in Foxglove/RViz-friendly message types."""

import math

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point
from hunav_msgs.msg import Agents
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray


BEHAVIOR_NAMES = {
    1: "regular",
    2: "impassive",
    3: "surprised",
    4: "scared",
    5: "curious",
    6: "threatening",
}

BEHAVIOR_COLORS = {
    1: (0.18, 0.55, 0.95),
    2: (0.55, 0.60, 0.65),
    3: (1.00, 0.78, 0.12),
    4: (0.72, 0.34, 0.95),
    5: (0.10, 0.82, 0.76),
    6: (0.95, 0.18, 0.16),
}


def color(red, green, blue, alpha=1.0):
    value = ColorRGBA()
    value.r = float(red)
    value.g = float(green)
    value.b = float(blue)
    value.a = float(alpha)
    return value


def point(x, y, z=0.0):
    value = Point()
    value.x = float(x)
    value.y = float(y)
    value.z = float(z)
    return value


def duration(seconds):
    value = DurationMsg()
    nanoseconds = max(0, int(round(seconds * 1_000_000_000)))
    value.sec = nanoseconds // 1_000_000_000
    value.nanosec = nanoseconds % 1_000_000_000
    return value


def base_marker(header, namespace, marker_id, marker_type, lifetime_seconds):
    marker = Marker()
    marker.header = header
    marker.ns = namespace
    marker.id = int(marker_id)
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.frame_locked = True
    marker.lifetime = duration(lifetime_seconds)
    return marker


def build_human_markers(
    message,
    marker_stamp,
    measurement_age,
    prediction_horizon,
    prediction_dt,
    robot_radius,
    safe_distance,
    geometry_uncertainty,
    body_height,
    marker_lifetime,
):
    """Convert one HuNav snapshot into stable-ID visualization markers."""
    output = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    output.markers.append(clear)

    header = Header()
    header.frame_id = message.header.frame_id
    header.stamp = marker_stamp
    prediction_steps = max(1, int(math.ceil(prediction_horizon / prediction_dt)))
    age = max(0.0, measurement_age)

    for index, agent in enumerate(message.agents):
        marker_id = int(agent.id) if agent.id >= 0 else index
        behavior = int(agent.behavior.type)
        behavior_name = BEHAVIOR_NAMES.get(behavior, f"behavior_{behavior}")
        rgb = BEHAVIOR_COLORS.get(behavior, (0.95, 0.95, 0.95))
        velocity_x = float(agent.velocity.linear.x)
        velocity_y = float(agent.velocity.linear.y)
        source_x = float(agent.position.position.x)
        source_y = float(agent.position.position.y)
        radius = float(agent.radius)
        if not all(
            math.isfinite(value)
            for value in (source_x, source_y, velocity_x, velocity_y, radius)
        ) or radius <= 0.0:
            continue
        speed = math.hypot(velocity_x, velocity_y)
        current_x = source_x + age * velocity_x
        current_y = source_y + age * velocity_y
        exclusion_radius = radius + robot_radius + geometry_uncertainty + safe_distance

        safety = base_marker(
            header, "human_mpc_exclusion", marker_id, Marker.CYLINDER, marker_lifetime
        )
        safety.pose.position = point(current_x, current_y, 0.01)
        safety.scale.x = 2.0 * exclusion_radius
        safety.scale.y = 2.0 * exclusion_radius
        safety.scale.z = 0.02
        safety.color = color(*rgb, 0.16)
        output.markers.append(safety)

        body = base_marker(
            header, "human_body", marker_id, Marker.CYLINDER, marker_lifetime
        )
        body.pose.position = point(current_x, current_y, 0.5 * body_height)
        body.scale.x = 2.0 * radius
        body.scale.y = 2.0 * radius
        body.scale.z = body_height
        body.color = color(*rgb, 0.88)
        output.markers.append(body)

        prediction = base_marker(
            header, "human_prediction", marker_id, Marker.LINE_STRIP, marker_lifetime
        )
        prediction.scale.x = 0.045
        prediction.color = color(*rgb, 0.85)
        for step in range(prediction_steps + 1):
            prediction_time = age + min(prediction_horizon, step * prediction_dt)
            prediction.points.append(
                point(
                    source_x + prediction_time * velocity_x,
                    source_y + prediction_time * velocity_y,
                    0.08,
                )
            )
        output.markers.append(prediction)

        velocity = base_marker(
            header, "human_velocity", marker_id, Marker.ARROW, marker_lifetime
        )
        velocity.scale.x = 0.045
        velocity.scale.y = 0.09
        velocity.scale.z = 0.12
        velocity.color = color(*rgb, 1.0)
        velocity.points = [
            point(current_x, current_y, body_height + 0.05),
            point(current_x + velocity_x, current_y + velocity_y, body_height + 0.05),
        ]
        output.markers.append(velocity)

        label = base_marker(
            header, "human_label", marker_id, Marker.TEXT_VIEW_FACING, marker_lifetime
        )
        label.pose.position = point(current_x, current_y, body_height + 0.35)
        label.scale.z = 0.22
        label.color = color(1.0, 1.0, 1.0, 1.0)
        label.text = f"ID {agent.id} | {behavior_name} | {speed:.2f} m/s"
        output.markers.append(label)

        if agent.goals:
            goals = base_marker(
                header, "human_goals", marker_id, Marker.SPHERE_LIST, marker_lifetime
            )
            goals.scale.x = 0.14
            goals.scale.y = 0.14
            goals.scale.z = 0.14
            goals.color = color(*rgb, 0.75)
            goals.points = [
                point(goal.position.x, goal.position.y, 0.07)
                for goal in agent.goals
                if math.isfinite(goal.position.x) and math.isfinite(goal.position.y)
            ]
            if goals.points:
                output.markers.append(goals)

    return output


class MpcVisualizer(Node):
    def __init__(self):
        super().__init__("mpc_visualizer")
        self.declare_parameter("global_plan_input_topic", "/plan")
        self.declare_parameter("local_trajectory_input_topic", "/FollowPath/predicted_path")
        self.declare_parameter("human_input_topic", "/human_states")
        self.declare_parameter("global_plan_output_topic", "/mpc/global_plan")
        self.declare_parameter("local_trajectory_output_topic", "/mpc/local_trajectory")
        self.declare_parameter("human_markers_output_topic", "/mpc/human_markers")
        self.declare_parameter("prediction_horizon", 2.5)
        self.declare_parameter("prediction_dt", 0.1)
        self.declare_parameter("robot_circumscribed_radius", 0.326)
        self.declare_parameter("safe_distance", 0.35)
        self.declare_parameter("geometry_uncertainty", 0.05)
        self.declare_parameter("human_body_height", 1.7)
        self.declare_parameter("marker_lifetime", 0.6)
        self.declare_parameter("maximum_measurement_age", 0.3)

        self.prediction_horizon = float(self.get_parameter("prediction_horizon").value)
        self.prediction_dt = float(self.get_parameter("prediction_dt").value)
        self.robot_radius = float(self.get_parameter("robot_circumscribed_radius").value)
        self.safe_distance = float(self.get_parameter("safe_distance").value)
        self.geometry_uncertainty = float(self.get_parameter("geometry_uncertainty").value)
        self.body_height = float(self.get_parameter("human_body_height").value)
        self.marker_lifetime = float(self.get_parameter("marker_lifetime").value)
        self.maximum_measurement_age = float(
            self.get_parameter("maximum_measurement_age").value
        )
        if self.prediction_horizon <= 0.0 or self.prediction_dt <= 0.0:
            raise ValueError("prediction_horizon and prediction_dt must be positive")

        volatile_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.global_plan_publisher = self.create_publisher(
            Path, self.get_parameter("global_plan_output_topic").value, latched_qos
        )
        self.local_trajectory_publisher = self.create_publisher(
            Path, self.get_parameter("local_trajectory_output_topic").value, volatile_qos
        )
        self.human_markers_publisher = self.create_publisher(
            MarkerArray, self.get_parameter("human_markers_output_topic").value, volatile_qos
        )
        self.create_subscription(
            Path,
            self.get_parameter("global_plan_input_topic").value,
            self.on_global_plan,
            volatile_qos,
        )
        self.create_subscription(
            Path,
            self.get_parameter("local_trajectory_input_topic").value,
            self.on_local_trajectory,
            volatile_qos,
        )
        self.create_subscription(
            Agents,
            self.get_parameter("human_input_topic").value,
            self.on_humans,
            volatile_qos,
        )
        self.get_logger().info(
            "Publishing /mpc/global_plan, /mpc/local_trajectory, and /mpc/human_markers"
        )

    def on_global_plan(self, message):
        self.global_plan_publisher.publish(message)

    def on_local_trajectory(self, message):
        self.local_trajectory_publisher.publish(message)

    def on_humans(self, message):
        now = self.get_clock().now()
        measurement_age = 0.0
        if message.header.stamp.sec != 0 or message.header.stamp.nanosec != 0:
            measured = Time.from_msg(message.header.stamp, clock_type=now.clock_type)
            measurement_age = min(
                self.maximum_measurement_age,
                max(0.0, (now - measured).nanoseconds / 1_000_000_000.0),
            )
        markers = build_human_markers(
            message=message,
            marker_stamp=now.to_msg(),
            measurement_age=measurement_age,
            prediction_horizon=self.prediction_horizon,
            prediction_dt=self.prediction_dt,
            robot_radius=self.robot_radius,
            safe_distance=self.safe_distance,
            geometry_uncertainty=self.geometry_uncertainty,
            body_height=self.body_height,
            marker_lifetime=self.marker_lifetime,
        )
        self.human_markers_publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = MpcVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
