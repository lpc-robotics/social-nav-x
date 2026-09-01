"""Controlled full-simulation acceptance driver for the V1 social automaton."""

from __future__ import annotations

import json
import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from hunav_msgs.msg import Agents
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import String


SCENARIOS = {
    "safe": (0.15, "CURIOUS", 5),
    "sudden": (0.30, "SURPRISED", 3),
    "fast": (0.80, "SCARED", 4),
}


def _angle_error(first: float, second: float) -> float:
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


class FormalScenarioVerifier(Node):
    """Publish one controlled velocity and verify the committed ROS evidence."""

    def __init__(self) -> None:
        super().__init__("verify_formal_social_scenario")
        self.declare_parameter("scenario", "safe")
        self.declare_parameter("round", 1)
        self.scenario = str(self.get_parameter("scenario").value)
        self.round_index = int(self.get_parameter("round").value)
        if self.scenario not in SCENARIOS:
            raise RuntimeError(
                f"scenario must be one of {sorted(SCENARIOS)}, got {self.scenario!r}"
            )
        if self.round_index < 1:
            raise RuntimeError("round must be positive")

        reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(
            String,
            "/formal_social_behavior/states",
            self._on_state,
            reliable,
        )
        self.create_subscription(
            String,
            "/formal_social_behavior/transitions",
            self._on_transition,
            reliable,
        )
        self.create_subscription(Agents, "/human_states", self._on_agents, 10)
        self.create_subscription(
            Odometry, "/odom", self._on_odom, qos_profile_sensor_data
        )

        self.state = None
        self.latest_observation = None
        self.transitions = []
        self.robot_xy = None
        self.odom_samples = deque(maxlen=400)
        self.human = None
        self.behavior_types = set()
        self.min_distance = math.inf
        self.max_distance = 0.0
        self.tracked_state = None
        self.tracked_first_distance = None
        self.tracked_last_distance = None
        self.tracked_min_distance = math.inf
        self.tracked_max_rise = 0.0
        self.min_scared_closing_speed = math.inf
        self.max_scared_outward_speed = -math.inf
        self.surprised_yaw_change = 0.0
        self.surprised_final_speed = math.inf
        self.surprised_facing_error = math.inf
        self.validation_errors = []
        self.reset_counts = set()

    def _record_validation_error(self, message: str) -> None:
        if message not in self.validation_errors:
            self.validation_errors.append(message)

    @staticmethod
    def _require_finite(label: str, values) -> None:
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError(f"{label} contains NaN/Inf")

    def _validate_observation(self, payload: dict) -> None:
        if int(payload["schema_version"]) != 1:
            raise ValueError("unsupported formal-social schema")
        if int(payload["sim_time_ns"]) < 0:
            raise ValueError("negative simulation stamp")
        if int(payload["agent_id"]) != 1:
            raise ValueError("unexpected formal target agent")
        if str(payload["state"]) not in {
            "NORMAL",
            "ATTENTION",
            "CURIOUS",
            "SURPRISED",
            "SCARED",
        }:
            raise ValueError("unknown formal state")
        events = payload["events"]
        known_events = {
            "ROBOT_VISIBLE",
            "ROBOT_LOST",
            "ROBOT_NEAR",
            "ROBOT_SAFE_APPROACH",
            "SUDDEN_NEAR",
            "PERSONAL_SPACE_VIOLATION",
            "ROBOT_FAST_APPROACH",
            "TTC_LOW",
            "ROBOT_LEAVING",
            "TIME_RESET",
        }
        if (
            not isinstance(events, list)
            or any(not isinstance(event, str) for event in events)
            or len(events) != len(set(events))
            or not set(events).issubset(known_events)
        ):
            raise ValueError("invalid formal event list")
        self._require_finite(
            "formal observation",
            (
                payload["distance"],
                payload["closing_speed"],
                payload["bearing_to_robot"],
                payload["relative_bearing"],
            ),
        )
        ttc = payload["ttc_seconds"]
        if ttc is not None and (
            not math.isfinite(float(ttc)) or float(ttc) < 0.0
        ):
            raise ValueError("invalid TTC")
        reset_count = int(payload["reset_count"])
        if reset_count < 0:
            raise ValueError("negative reset count")
        self.reset_counts.add(reset_count)

    def _on_state(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self._validate_observation(payload)
        except Exception as exc:
            self._record_validation_error(f"invalid state payload: {exc}")
            return
        self.state = str(payload["state"])
        self.latest_observation = payload
        if self.state == "SCARED":
            self._record_scared_kinematics(payload)
        distance = float(payload["distance"])
        self.min_distance = min(self.min_distance, distance)
        self.max_distance = max(self.max_distance, distance)
        if self.state == self.tracked_state:
            if self.tracked_first_distance is None:
                self.tracked_first_distance = distance
            self.tracked_last_distance = distance
            self.tracked_min_distance = min(self.tracked_min_distance, distance)
            self.tracked_max_rise = max(
                self.tracked_max_rise,
                distance - self.tracked_min_distance,
            )

    def _on_transition(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self._validate_observation(payload)
            if payload.get("message_type") != "transition":
                raise ValueError("transition message_type missing")
        except Exception as exc:
            self._record_validation_error(
                f"invalid transition payload: {exc}"
            )
            return
        self.transitions.append(payload)
        if payload.get("old_state") == "SCARED":
            self._record_scared_kinematics(payload)

    def _record_scared_kinematics(self, payload: dict) -> None:
        """Accumulate SCARED separation from stamp-matched state and odom."""

        closing_speed = float(payload["closing_speed"])
        self.min_scared_closing_speed = min(
            self.min_scared_closing_speed,
            closing_speed,
        )
        if not self.odom_samples:
            return
        observation_stamp = int(payload["sim_time_ns"])
        stamp_ns, robot_vx, robot_vy = min(
            self.odom_samples,
            key=lambda sample: abs(sample[0] - observation_stamp),
        )
        if abs(stamp_ns - observation_stamp) > 100_000_000:
            return
        bearing = float(payload["bearing_to_robot"])
        unit_x = math.cos(bearing)
        unit_y = math.sin(bearing)
        robot_toward_human = -(unit_x * robot_vx + unit_y * robot_vy)
        outward_speed = robot_toward_human - closing_speed
        self.max_scared_outward_speed = max(
            self.max_scared_outward_speed,
            outward_speed,
        )

    def _on_agents(self, msg: Agents) -> None:
        for agent in msg.agents:
            if int(agent.id) == 1:
                try:
                    pose = agent.position
                    velocity = agent.velocity
                    values = (
                        pose.position.x,
                        pose.position.y,
                        pose.position.z,
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                        velocity.linear.x,
                        velocity.linear.y,
                        velocity.linear.z,
                        velocity.angular.x,
                        velocity.angular.y,
                        velocity.angular.z,
                        agent.yaw,
                        agent.radius,
                    )
                    self._require_finite("human state", values)
                except Exception as exc:
                    self._record_validation_error(
                        f"invalid human state: {exc}"
                    )
                    return
                self.human = agent
                self.behavior_types.add(int(agent.behavior.type))
                if int(agent.behavior.type) == 4 and self.robot_xy is not None:
                    dx = float(agent.position.position.x) - self.robot_xy[0]
                    dy = float(agent.position.position.y) - self.robot_xy[1]
                    distance = math.hypot(dx, dy)
                    if distance > 1e-9:
                        outward_speed = (
                            float(agent.velocity.linear.x) * dx
                            + float(agent.velocity.linear.y) * dy
                        ) / distance
                        self.max_scared_outward_speed = max(
                            self.max_scared_outward_speed,
                            outward_speed,
                        )
                break

    def _on_odom(self, msg: Odometry) -> None:
        try:
            pose = msg.pose.pose
            velocity = msg.twist.twist
            self._require_finite(
                "odometry",
                (
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                    velocity.linear.x,
                    velocity.linear.y,
                    velocity.linear.z,
                    velocity.angular.x,
                    velocity.angular.y,
                    velocity.angular.z,
                ),
            )
        except Exception as exc:
            self._record_validation_error(f"invalid odometry: {exc}")
            return
        self.robot_xy = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )
        quaternion = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0
            * (
                float(quaternion.w) * float(quaternion.z)
                + float(quaternion.x) * float(quaternion.y)
            ),
            1.0
            - 2.0
            * (
                float(quaternion.y) * float(quaternion.y)
                + float(quaternion.z) * float(quaternion.z)
            ),
        )
        body_vx = float(msg.twist.twist.linear.x)
        body_vy = float(msg.twist.twist.linear.y)
        map_vx = math.cos(yaw) * body_vx - math.sin(yaw) * body_vy
        map_vy = math.sin(yaw) * body_vx + math.cos(yaw) * body_vy
        stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )
        self.odom_samples.append((stamp_ns, map_vx, map_vy))

    def publish_velocity(self, linear: float) -> None:
        command = Twist()
        command.linear.x = float(linear)
        self._cmd_pub.publish(command)

    def latest_robot_toward_human_speed(self) -> float:
        """Project the latest map-frame odometry toward the target human."""

        if not self.odom_samples or self.robot_xy is None or self.human is None:
            return -math.inf
        _, velocity_x, velocity_y = self.odom_samples[-1]
        offset_x = float(self.human.position.position.x) - self.robot_xy[0]
        offset_y = float(self.human.position.position.y) - self.robot_xy[1]
        distance = math.hypot(offset_x, offset_y)
        if distance <= 1e-9:
            return math.inf
        return (
            velocity_x * offset_x + velocity_y * offset_y
        ) / distance

    def spin_until(
        self,
        predicate,
        timeout: float,
        *,
        linear: float = 0.0,
        on_success_linear: float | None = None,
    ) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_velocity(linear)
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                if on_success_linear is not None:
                    self.publish_velocity(on_success_linear)
                return True
        return False

    def pulse_until(
        self,
        predicate,
        duration: float,
        *,
        linear: float,
        on_success_linear: float = 0.0,
    ) -> bool:
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_velocity(linear)
            rclpy.spin_once(self, timeout_sec=0.005)
            if predicate():
                self.publish_velocity(on_success_linear)
                return True
        self.publish_velocity(0.0)
        return False

    def stop(self) -> None:
        for _ in range(12):
            self.publish_velocity(0.0)
            rclpy.spin_once(self, timeout_sec=0.05)

    def transition_to(self, state: str, *, after: int) -> dict | None:
        for transition in self.transitions[after:]:
            if transition.get("new_state") == state:
                return transition
        return None

    def target_behavior_observed(self, behavior_type: int) -> bool:
        return behavior_type in self.behavior_types

    def scared_separation_observed(
        self,
        *,
        after: int,
    ) -> bool:
        transition_separation = any(
            transition.get("old_state") == "SCARED"
            and float(transition["closing_speed"]) <= -0.01
            for transition in self.transitions[after:]
        )
        state_separation = self.tracked_max_rise >= 0.002
        return (
            (transition_separation or state_separation)
            and self.min_scared_closing_speed <= -0.01
            and self.max_scared_outward_speed >= 0.01
        )

    def track_state_distance(self, state: str) -> None:
        self.tracked_state = state
        self.tracked_first_distance = None
        self.tracked_last_distance = None
        self.tracked_min_distance = math.inf
        self.tracked_max_rise = 0.0

    def surprised_response_observed(self, initial_yaw: float) -> bool:
        if self.human is None or self.robot_xy is None:
            return False
        speed = math.hypot(
            float(self.human.velocity.linear.x),
            float(self.human.velocity.linear.y),
        )
        dx = self.robot_xy[0] - float(self.human.position.position.x)
        dy = self.robot_xy[1] - float(self.human.position.position.y)
        target_yaw = math.atan2(dy, dx)
        current_yaw = float(self.human.yaw)
        self.surprised_yaw_change = _angle_error(current_yaw, initial_yaw)
        self.surprised_final_speed = speed
        self.surprised_facing_error = _angle_error(current_yaw, target_yaw)
        return (
            speed <= 0.08
            and self.surprised_facing_error <= 0.35
            and self.surprised_yaw_change >= 0.20
        )


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FormalScenarioVerifier()
    speed, target_state, behavior_type = SCENARIOS[node.scenario]
    try:
        def scenario_ready() -> bool:
            base_ready = node.robot_xy is not None and node.human is not None
            if not base_ready or node.latest_observation is None:
                return False
            return node.state == "ATTENTION"

        ready = node.spin_until(scenario_ready, 45.0)
        _require(
            ready,
            "formal topics did not reach the scenario start state: "
            f"state={node.state} robot={node.robot_xy} human={node.human is not None}",
        )
        _require(
            node.count_publishers("/cmd_vel") == 1,
            "controlled acceptance requires exactly one /cmd_vel publisher; "
            f"found {node.count_publishers('/cmd_vel')}",
        )
        _require(
            not node.validation_errors,
            f"invalid startup telemetry: {node.validation_errors}",
        )
        start_distance = float(node.latest_observation["distance"])
        start_human_yaw = float(node.human.yaw)
        transition_start = len(node.transitions)
        node.track_state_distance(target_state)

        def target_seen() -> bool:
            return (
                node.transition_to(target_state, after=transition_start)
                is not None
            )

        if node.scenario == "fast":
            # Observe the commanded velocity in real odometry, then pre-brake
            # while the corresponding HuNav request is in flight. Waiting for
            # the transition topic before braking leaves one additional
            # physics step of forward travel: Scared turns away in that same
            # compute beat and ROBOT_LOST is valid on the next beat. Bounded
            # retries remove scheduler-phase dependence while the transition
            # still has to record a genuine >=0.5 m/s closing sample below.
            reached = False
            for _ in range(8):
                sampled_or_reached = node.spin_until(
                    lambda: target_seen()
                    or node.latest_robot_toward_human_speed() >= 0.70,
                    0.5,
                    linear=speed,
                    on_success_linear=-speed,
                )
                if not sampled_or_reached:
                    continue
                if target_seen():
                    reached = True
                    break
                reached = node.spin_until(
                    target_seen,
                    0.25,
                    linear=-speed,
                    on_success_linear=-speed,
                )
                if reached:
                    break
                node.spin_until(
                    lambda: abs(node.latest_robot_toward_human_speed()) <= 0.05,
                    0.5,
                    linear=0.0,
                )
        else:
            reached = node.spin_until(target_seen, 20.0, linear=speed)
        target_transition = node.transition_to(
            target_state, after=transition_start
        )
        _require(
            reached and target_transition is not None,
            f"{node.scenario} did not reach {target_state}; "
            f"state={node.state} transitions={node.transitions[transition_start:]}",
        )
        _require(
            target_transition["old_state"] == "ATTENTION",
            f"unexpected source state: {target_transition}",
        )
        _require(
            int(target_transition["reset_count"]) == 1,
            f"target profile did not use exactly one reset: {target_transition}",
        )
        expected_causes = {
            "safe": "ATTENTION_DWELL",
            "sudden": "SUDDEN_NEAR",
            "fast": "ROBOT_FAST_APPROACH",
        }
        _require(
            target_transition["cause"] == expected_causes[node.scenario],
            f"unexpected target cause: {target_transition}",
        )
        target_closing_speed = float(target_transition["closing_speed"])
        closing_speed_ok = {
            "safe": 0.10 <= target_closing_speed < 0.25,
            "sudden": 0.25 <= target_closing_speed < 0.50,
            "fast": target_closing_speed >= 0.50,
        }[node.scenario]
        _require(
            closing_speed_ok,
            "target transition did not observe the controlled approach speed: "
            f"scenario={node.scenario} closing_speed={target_closing_speed:.6f}",
        )
        if node.scenario == "sudden":
            active_events = set(target_transition["events"])
            _require(
                not active_events.intersection(
                    {
                        "PERSONAL_SPACE_VIOLATION",
                        "TTC_LOW",
                        "ROBOT_FAST_APPROACH",
                    }
                ),
                f"sudden-near transition was already dangerous: {target_transition}",
            )
        target_distance = float(target_transition["distance"])
        response_linear = 0.0
        if node.scenario == "fast":
            # Begin the plan's leaving phase immediately after the safety
            # transition. This both arrests DDS command latency and lets the
            # next committed SCARED sample prove increasing total separation.
            response_linear = -abs(speed)
        else:
            node.stop()

        response_ok = node.spin_until(
            lambda: node.target_behavior_observed(behavior_type),
            4.0,
            linear=response_linear,
        )
        _require(
            response_ok,
            f"HuNav behavior type {behavior_type} was not observed",
        )

        if node.scenario == "safe":
            distance_response = node.spin_until(
                lambda: node.tracked_first_distance is not None
                and node.tracked_last_distance
                <= node.tracked_first_distance - 0.002,
                4.0,
            )
            _require(
                distance_response,
                "Curious did not reduce human/robot distance: "
                f"first={node.tracked_first_distance} "
                f"last={node.tracked_last_distance}",
            )
        elif node.scenario == "sudden":
            stopped_and_facing = node.spin_until(
                lambda: node.surprised_response_observed(start_human_yaw),
                4.0,
            )
            _require(
                stopped_and_facing,
                "Surprised did not stop, change yaw, and face the robot: "
                f"yaw_change={node.surprised_yaw_change:.6f} "
                f"speed={node.surprised_final_speed:.6f} "
                f"facing_error={node.surprised_facing_error:.6f}",
            )
        else:
            distance_response = node.spin_until(
                lambda: node.scared_separation_observed(
                    after=transition_start,
                ),
                6.0,
                linear=response_linear,
            )
            _require(
                distance_response,
                "Scared did not produce outward-separation evidence: "
                f"first={node.tracked_first_distance} "
                f"last={node.tracked_last_distance} "
                f"max_rise={node.tracked_max_rise:.6f} "
                f"min_closing={node.min_scared_closing_speed:.6f} "
                f"human_outward_speed={node.max_scared_outward_speed:.6f} "
                f"transitions={node.transitions[transition_start:]}",
            )

        # A front-facing Scared action turns away from the robot and may
        # legitimately recover through ROBOT_LOST before this point. Search
        # from the target transition, while still driving a leaving command
        # for scenarios whose recovery has not happened yet.
        recovery_start = transition_start
        _require(
            node.count_publishers("/cmd_vel") == 1,
            "another /cmd_vel publisher appeared during acceptance",
        )
        recovered = node.spin_until(
            lambda: node.transition_to("NORMAL", after=recovery_start)
            is not None,
            15.0,
            linear=-math.copysign(0.80, speed),
        )
        node.stop()
        recovery_transition = node.transition_to(
            "NORMAL", after=recovery_start
        )
        _require(
            recovered and recovery_transition is not None,
            f"{node.scenario} did not recover to NORMAL; state={node.state}",
        )
        _require(
            int(recovery_transition["reset_count"]) == 2,
            "Regular recovery did not produce exactly the second reset: "
            f"{recovery_transition}",
        )
        _require(
            max(node.reset_counts, default=-1) == 2
            and not any(value > 2 for value in node.reset_counts),
            f"unexpected or duplicate resets: {sorted(node.reset_counts)}",
        )
        recovery_distance = float(recovery_transition["distance"])
        if node.scenario == "fast":
            # The sign must be a measured increase, not an inferred command.
            # A 0.1 mm floor only rejects floating-point equality/noise; the
            # independent closing-speed and human-outward-speed gates above
            # carry the dynamic-response margin.
            _require(
                recovery_distance >= target_distance + 0.0001,
                "Scared interval did not show an actual distance increase: "
                f"target={target_distance:.6f} recovery={recovery_distance:.6f}",
            )
        transition_keys = [
            (item["sim_time_ns"], item["old_state"], item["new_state"])
            for item in node.transitions
        ]
        _require(
            len(transition_keys) == len(set(transition_keys)),
            f"duplicate committed transitions: {transition_keys}",
        )
        _require(
            not node.validation_errors,
            f"invalid runtime telemetry: {node.validation_errors}",
        )

        path = [
            f"{item['old_state']}->{item['new_state']}:{item['cause']}"
            for item in node.transitions[transition_start:]
        ]
        response_details = ""
        if node.scenario == "sudden":
            response_details = (
                " surprised_yaw_change="
                f"{node.surprised_yaw_change:.6f}"
                " surprised_speed="
                f"{node.surprised_final_speed:.6f}"
                " surprised_facing_error="
                f"{node.surprised_facing_error:.6f}"
            )
        elif node.scenario == "fast":
            response_details = (
                " scared_distance_rise="
                f"{recovery_distance - target_distance:.6f}"
                " scared_exit_closing_speed="
                f"{float(recovery_transition['closing_speed']):.6f}"
                " human_outward_speed="
                f"{node.max_scared_outward_speed:.6f}"
            )
        print(
            "FORMAL_SOCIAL_SCENARIO_OK "
            f"scenario={node.scenario} round={node.round_index} "
            f"speed={abs(speed):.2f} target={target_state} "
            f"start_distance={start_distance:.3f} "
            f"target_distance={target_distance:.3f} "
            f"target_closing_speed={target_closing_speed:.6f} "
            f"recovery_distance={recovery_distance:.3f} "
            f"resets={int(recovery_transition['reset_count'])} "
            f"cmd_vel_publishers={node.count_publishers('/cmd_vel')} "
            f"behavior_types={','.join(str(value) for value in sorted(node.behavior_types))} "
            f"path={'|'.join(path)}"
            f"{response_details}"
        )
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
