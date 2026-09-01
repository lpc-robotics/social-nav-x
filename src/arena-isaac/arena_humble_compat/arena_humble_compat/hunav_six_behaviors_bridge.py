import copy
import math
import os
import time

import rclpy
import yaml
from arena_humble_compat.bridge_validation import (
    STRICT_SIX_BEHAVIOR_TYPES,
    calculate_runtime_rates,
    character_model_mapping,
    format_ready_status,
    format_runtime_status,
    require_plain_integer,
    validate_agent_definitions,
)
from arena_people_msgs.msg import Pedestrian, SpawnPedestrian
from arena_people_msgs.srv import SpawnPedestrians, UpdatePedestrians
from geometry_msgs.msg import Pose, Twist
from hunav_msgs.msg import Agent, Agents
from hunav_msgs.srv import ComputeAgents
from isaacsim_msgs.msg import Wall
from isaacsim_msgs.srv import SpawnUrdf, SpawnWalls
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


WHEEL_JOINTS = [
    "front_left_wheel_joint",
    "rear_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_right_wheel_joint",
]

BEHAVIOR_NAMES = {
    1: "REGULAR",
    2: "IMPASSIVE",
    3: "SURPRISED",
    4: "SCARED",
    5: "CURIOUS",
    6: "THREATENING",
}

DEFAULT_CHARACTER_MODELS = [
    "original_female_adult_business_02",
    "original_female_adult_medical_01",
    "original_male_adult_medical_01",
    "original_male_adult_construction_01",
    "original_male_adult_construction_05",
    "original_female_adult_police_01",
]


def _yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _set_yaw(quaternion, yaw: float):
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = math.sin(yaw / 2.0)
    quaternion.w = math.cos(yaw / 2.0)


class ArenaHuNavIsaacBridge(Node):
    """Adapt HuNav's simulator-agnostic output to Arena Isaac 5.1 services."""

    def __init__(self):
        super().__init__("arena_hunav_isaac_bridge")
        self.declare_parameter("agent_config_file", "")
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("update_rate", 40.0)
        self.declare_parameter("display_rate", 5.0)
        self.declare_parameter("max_integration_step", 0.025)
        self.declare_parameter("scheduler_rate", 100.0)
        self.declare_parameter("robot_x", 3.0)
        self.declare_parameter("robot_y", 3.0)
        # Match Arena's Jackal diff_drive_controller calibration. The physical
        # track alone under-commands a four-wheel skid-steer turn because the
        # tyres must also slide laterally.
        self.declare_parameter("wheel_radius", 0.098)
        self.declare_parameter("wheel_separation", 0.36)
        self.declare_parameter("wheel_separation_multiplier", 1.5)
        # Isaac PhysX uses isotropic wheel friction. A four-wheel skid-steer
        # therefore has a yaw-rate dead zone that the Gazebo calibration alone
        # cannot remove. These measured feed-forward terms preserve the
        # official effective track while compensating that contact loss.
        self.declare_parameter("angular_velocity_gain", 1.18)
        self.declare_parameter("angular_static_compensation", 0.46)
        self.declare_parameter("angular_compensation_transition", 0.1)
        self.declare_parameter("character_models", DEFAULT_CHARACTER_MODELS)
        self.declare_parameter("strict_six_behavior_demo", True)
        self.declare_parameter("status_topic", "/arena5/six_behaviors/status")
        self.declare_parameter(
            "agent_debug_topic", "/arena5/six_behaviors/agents"
        )
        self.declare_parameter("ready_marker", "SIX_BEHAVIORS_READY")

        self._strict_six_behavior_demo = bool(
            self.get_parameter("strict_six_behavior_demo").value
        )
        self._ready_marker = str(self.get_parameter("ready_marker").value)
        if not self._ready_marker:
            raise RuntimeError("ready_marker must be non-empty")

        self._agents, self._agent_templates = self._load_agents()
        self._agent_count = len(self._agents.agents)
        self._models = self._load_character_models()

        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self._agent_debug_pub = self.create_publisher(
            Agents, str(self.get_parameter("agent_debug_topic").value), 10
        )
        self._joint_pub = self.create_publisher(
            JointState, "/isaac/joint_commands_velocity", 10
        )
        self.create_subscription(Twist, "/cmd_vel", self._on_twist, 10)
        self.create_subscription(
            Odometry, "/odom", self._on_odom, qos_profile_sensor_data
        )

        self._walls = self.create_client(SpawnWalls, "/isaac/SpawnWalls")
        self._robot = self.create_client(SpawnUrdf, "/isaac/SpawnUrdf")
        self._spawn_people = self.create_client(
            SpawnPedestrians, "/isaac/SpawnPedestrians"
        )
        self._update_people = self.create_client(
            UpdatePedestrians, "/isaac/UpdatePedestrians"
        )
        self._pause = self.create_client(Trigger, "/isaac/PauseSimulation")
        self._unpause = self.create_client(Trigger, "/isaac/UnpauseSimulation")
        self._compute = self.create_client(ComputeAgents, "/compute_agents")

        self._last_twist = Twist()
        self._last_twist_monotonic = 0.0
        self._odom = None
        self._odom_time_ns = None
        self._odom_xy_yaw = None
        self._robot_velocity = (0.0, 0.0, 0.0)
        self._compute_future = None
        self._compute_stamp = None
        self._compute_stamp_ns = None
        self._last_integrated_stamp_ns = None
        self._update_future = None
        self._display_snapshot = None
        self._needs_isaac_update = False
        self._last_display_dispatch_monotonic = 0.0
        self._compute_count = 0
        self._update_count = 0
        self._substep_count = 0
        self._max_integration_step_seen = 0.0
        self._metrics_start_monotonic = None
        self._last_report_monotonic = None
        self._last_reported_compute_count = 0
        self._last_reported_update_count = 0
        self._ready_reported = False
        self._ideal_chassis = str(
            os.environ.get("ARENA_IDEAL_CHASSIS", "false")
        ).lower() in ("true", "1")

        update_rate = float(self.get_parameter("update_rate").value)
        display_rate = float(self.get_parameter("display_rate").value)
        max_step = float(self.get_parameter("max_integration_step").value)
        scheduler_rate = float(self.get_parameter("scheduler_rate").value)
        if update_rate <= 0.0:
            raise RuntimeError("update_rate must be positive")
        if display_rate <= 0.0:
            raise RuntimeError("display_rate must be positive")
        if max_step <= 0.0:
            raise RuntimeError("max_integration_step must be positive")
        if scheduler_rate < update_rate:
            raise RuntimeError("scheduler_rate must be at least update_rate")

        self._compute_rate = update_rate
        self._compute_period_ns = max(1, round(1_000_000_000 / update_rate))
        self._display_period = 1.0 / display_rate
        self._max_integration_step_ns = max(1, round(max_step * 1_000_000_000))
        self._status_interval = max(20, round(update_rate * 5.0))

        # A steady-clock scheduler keeps HuNav service progress independent of
        # Isaac's render/ROS clock cadence. ROS sim time is still used for every
        # integration stamp and is split into max_integration_step increments.
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self._publish_wheels, clock=self._steady_clock)
        self.create_timer(
            1.0 / scheduler_rate, self._tick, clock=self._steady_clock
        )
        self.get_logger().info(
            "SIX_BEHAVIORS_SCHEDULER "
            f"compute_rate={update_rate:.1f} display_rate={display_rate:.1f} "
            f"max_dt={max_step:.3f} scheduler_rate={scheduler_rate:.1f} "
            f"ideal_chassis={self._ideal_chassis}"
        )

    def _load_agents(self):
        config_path = str(self.get_parameter("agent_config_file").value)
        if not config_path:
            raise RuntimeError("agent_config_file parameter is required")
        with open(config_path, "r", encoding="utf-8") as config_file:
            document = yaml.safe_load(config_file)

        try:
            parameters = document["hunav_loader"]["ros__parameters"]
            agent_names = parameters["agents"]
        except (KeyError, TypeError) as exc:
            raise RuntimeError(f"invalid HuNav agent file: {config_path}") from exc

        container = Agents()
        container.header.frame_id = "map"
        templates = {}
        for name in agent_names:
            spec = parameters[name]
            behavior_spec = spec["behavior"]
            init_pose = spec["init_pose"]

            agent_id = require_plain_integer(spec["id"], f"{name}.id")
            behavior_type = require_plain_integer(
                behavior_spec["type"], f"{name}.behavior.type"
            )
            behavior_configuration = require_plain_integer(
                behavior_spec["configuration"],
                f"{name}.behavior.configuration",
            )

            agent = Agent()
            agent.id = agent_id
            agent.type = Agent.PERSON
            agent.skin = int(spec["skin"])
            agent.name = str(name)
            agent.group_id = int(spec["group_id"])
            agent.position.position.x = float(init_pose["x"])
            agent.position.position.y = float(init_pose["y"])
            agent.position.position.z = 0.0
            agent.yaw = float(init_pose["h"])
            _set_yaw(agent.position.orientation, agent.yaw)
            agent.desired_velocity = float(spec["max_vel"])
            agent.radius = float(spec["radius"])

            agent.behavior.type = behavior_type
            agent.behavior.state = 0
            agent.behavior.configuration = behavior_configuration
            agent.behavior.duration = float(behavior_spec["duration"])
            agent.behavior.once = bool(behavior_spec["once"])
            agent.behavior.vel = float(behavior_spec["vel"])
            agent.behavior.dist = float(behavior_spec["dist"])
            agent.behavior.goal_force_factor = float(
                behavior_spec["goal_force_factor"]
            )
            agent.behavior.obstacle_force_factor = float(
                behavior_spec["obstacle_force_factor"]
            )
            agent.behavior.social_force_factor = float(
                behavior_spec["social_force_factor"]
            )
            agent.behavior.other_force_factor = float(
                behavior_spec["other_force_factor"]
            )

            agent.goal_radius = float(spec["goal_radius"])
            agent.cyclic_goals = bool(spec["cyclic_goals"])
            for goal_name in spec["goals"]:
                goal_spec = spec[goal_name]
                goal = Pose()
                goal.position.x = float(goal_spec["x"])
                goal.position.y = float(goal_spec["y"])
                goal.position.z = 0.0
                goal.orientation.w = 1.0
                agent.goals.append(goal)

            container.agents.append(agent)
            templates[agent.name] = copy.deepcopy(agent)

        validate_agent_definitions(
            (
                (
                    agent.id,
                    agent.name,
                    agent.behavior.type,
                    agent.behavior.configuration,
                )
                for agent in container.agents
            ),
            self._strict_six_behavior_demo,
        )
        return container, templates

    def _load_character_models(self):
        model_names = list(self.get_parameter("character_models").value)
        agent_names = [agent.name for agent in self._agents.agents]
        return character_model_mapping(agent_names, model_names)

    def _status(self, text: str):
        self.get_logger().info(text)
        self._status_pub.publish(String(data=text))

    def _call(self, client, request, name: str, timeout: float = 180.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"timeout waiting for {name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError(f"timeout calling {name}")
        if future.exception() is not None:
            raise RuntimeError(f"{name} failed: {future.exception()}")
        return future.result()

    def spawn_scene(self):
        pause_result = self._call(
            self._pause, Trigger.Request(), "/isaac/PauseSimulation"
        )
        if not pause_result or not pause_result.success:
            raise RuntimeError(f"could not pause simulation: {pause_result}")

        walls = SpawnWalls.Request()
        corners = [
            ((0.0, 0.0), (30.0, 0.0)),
            ((30.0, 0.0), (30.0, 23.0)),
            ((30.0, 23.0), (0.0, 23.0)),
            ((0.0, 23.0), (0.0, 0.0)),
        ]
        for index, (start, end) in enumerate(corners):
            wall = Wall()
            wall.name = f"six_behaviors_wall_{index}"
            wall.start.x, wall.start.y, wall.start.z = start[0], start[1], 0.0
            wall.end.x, wall.end.y, wall.end.z = end[0], end[1], 2.5
            wall.thickness = 0.1
            walls.walls.append(wall)
        wall_result = self._call(self._walls, walls, "/isaac/SpawnWalls")
        if not wall_result or not all(wall_result.ret):
            raise RuntimeError(f"wall spawn failed: {wall_result}")
        self._status("SIX_BEHAVIORS_WALLS_OK count=4")

        request = SpawnPedestrians.Request()
        for agent in self._agents.agents:
            item = SpawnPedestrian()
            item.model_ref = self._models[agent.name]
            item.pedestrian.name = agent.name
            item.pedestrian.pose = copy.deepcopy(agent.position)
            request.pedestrians.append(item)
        result = self._call(
            self._spawn_people,
            request,
            "/isaac/SpawnPedestrians",
            timeout=300.0,
        )
        if not result or list(result.results) != [0] * self._agent_count:
            if self._strict_six_behavior_demo:
                raise RuntimeError(f"six-pedestrian spawn failed: {result}")
            raise RuntimeError(f"pedestrian spawn failed: {result}")
        mapping = ",".join(
            f"{agent.name}:{agent.behavior.type}"
            for agent in self._agents.agents
        )
        self._status(
            f"SIX_BEHAVIORS_SPAWN_OK count={self._agent_count} types={mapping}"
        )

        # SpawnUrdf performs the one World.reset() needed after stage edits.
        # Keep it last so all Character prims survive that reset.
        urdf_path = str(self.get_parameter("urdf_path").value)
        if not urdf_path:
            raise RuntimeError("urdf_path parameter is required")
        robot = SpawnUrdf.Request()
        robot.name = "arena_robot"
        robot.urdf_path = urdf_path
        robot.robot_model = "jackal"
        robot.base_frame = "base_link"
        robot.odom_frame = "odom"
        robot.pose.position.x = float(self.get_parameter("robot_x").value)
        robot.pose.position.y = float(self.get_parameter("robot_y").value)
        robot.pose.position.z = 0.15
        robot.pose.orientation.w = 1.0
        robot.cmd_vel_topic = "/cmd_vel"
        robot.joint_states_topic = "/joint_states"
        robot.odom_topic = "/odom"
        robot.localization = True
        robot_result = self._call(self._robot, robot, "/isaac/SpawnUrdf")
        if not robot_result or not robot_result.path:
            raise RuntimeError(f"robot spawn failed: {robot_result}")
        self._status(f"SIX_BEHAVIORS_ROBOT_OK path={robot_result.path}")

        if not self._compute.wait_for_service(timeout_sec=60.0):
            raise RuntimeError("timeout waiting for HuNav /compute_agents")
        if not self._update_people.wait_for_service(timeout_sec=60.0):
            raise RuntimeError("timeout waiting for Isaac pedestrian updates")

        unpause_result = self._call(
            self._unpause, Trigger.Request(), "/isaac/UnpauseSimulation"
        )
        if not unpause_result or not unpause_result.success:
            raise RuntimeError(f"could not unpause simulation: {unpause_result}")
        self._status("SIX_BEHAVIORS_HUNAV_SERVICE_OK service=/compute_agents")

    def _on_twist(self, msg: Twist):
        self._last_twist = msg
        self._last_twist_monotonic = time.monotonic()

    def _publish_wheels(self):
        command = self._last_twist
        if time.monotonic() - self._last_twist_monotonic > 0.5:
            command = Twist()
        wheel_radius = float(self.get_parameter("wheel_radius").value)
        wheel_separation = float(self.get_parameter("wheel_separation").value)
        angular = command.angular.z
        if self._ideal_chassis:
            # The D6 chassis follows raw cmd_vel directly. Wheel joints remain
            # for visualization/state only, so the old skid-steer dead-zone
            # feed-forward must not be applied a second time.
            compensated_angular = angular
        else:
            wheel_separation *= float(
                self.get_parameter("wheel_separation_multiplier").value
            )
            transition = max(
                1e-6,
                float(self.get_parameter("angular_compensation_transition").value),
            )
            compensated_angular = (
                float(self.get_parameter("angular_velocity_gain").value) * angular
                + float(self.get_parameter("angular_static_compensation").value)
                * math.tanh(angular / transition)
            )
        left = (
            command.linear.x - compensated_angular * wheel_separation / 2.0
        ) / wheel_radius
        right = (
            command.linear.x + compensated_angular * wheel_separation / 2.0
        ) / wheel_radius
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = WHEEL_JOINTS
        msg.velocity = [left, left, right, right]
        self._joint_pub.publish(msg)

    def _on_odom(self, msg: Odometry):
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        # nav_msgs/Odometry expresses twist in child_frame_id (base_link).
        # Rotate the official Isaac rigid-body velocity into the map/odom frame
        # expected by HuNav instead of differentiating and filtering pose.
        body_vx = msg.twist.twist.linear.x
        body_vy = msg.twist.twist.linear.y
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        self._robot_velocity = (
            cos_yaw * body_vx - sin_yaw * body_vy,
            sin_yaw * body_vx + cos_yaw * body_vy,
            msg.twist.twist.angular.z,
        )
        self._odom = msg
        self._odom_time_ns = stamp_ns
        self._odom_xy_yaw = (x, y, yaw)

    def _robot_agent(self):
        odom = self._odom
        robot = Agent()
        robot.id = 0
        robot.type = Agent.ROBOT
        robot.name = "arena_robot"
        robot.group_id = -1
        robot.position = copy.deepcopy(odom.pose.pose)
        robot.yaw = _yaw_from_quaternion(robot.position.orientation)
        vx, vy, wz = self._robot_velocity
        robot.velocity.linear.x = vx
        robot.velocity.linear.y = vy
        robot.velocity.angular.z = wz
        robot.linear_vel = math.hypot(vx, vy)
        robot.angular_vel = wz
        robot.desired_velocity = 1.0
        robot.radius = 0.35
        return robot

    def _restore_static_agent_fields(self, agents: Agents):
        for agent in agents.agents:
            template = self._agent_templates.get(agent.name)
            if template is None:
                continue
            agent.skin = template.skin
            agent.group_id = template.group_id
            agent.desired_velocity = template.desired_velocity
            agent.radius = template.radius
            agent.cyclic_goals = template.cyclic_goals
            agent.goal_radius = template.goal_radius

    def _finish_isaac_update(self):
        if self._update_future is None or not self._update_future.done():
            return

        if self._update_future.exception() is not None:
            self.get_logger().error(
                f"Isaac pedestrian update failed: {self._update_future.exception()}"
            )
            self._needs_isaac_update = True
        else:
            result = self._update_future.result()
            if result and list(result.results) == [0] * self._agent_count:
                self._update_count += 1
            else:
                self.get_logger().error(
                    f"Isaac pedestrian update returned errors: {result}"
                )
                self._needs_isaac_update = True
        self._update_future = None

    def _finish_hunav_compute(self):
        if self._compute_future is None or not self._compute_future.done():
            return

        if self._compute_future.exception() is not None:
            self.get_logger().error(
                f"HuNav compute failed: {self._compute_future.exception()}"
            )
            self._compute_future = None
            self._compute_stamp = None
            self._compute_stamp_ns = None
            return

        response = self._compute_future.result()
        invalid_response = response is None
        if response is not None:
            invalid_response = (
                len(response.updated_agents.agents) != self._agent_count
            )
        if invalid_response:
            self.get_logger().error(
                f"HuNav compute returned an invalid response: {response}"
            )
            self._compute_future = None
            self._compute_stamp = None
            self._compute_stamp_ns = None
            return

        previous_stamp_ns = self._last_integrated_stamp_ns
        self._last_integrated_stamp_ns = self._compute_stamp_ns
        if previous_stamp_ns is not None:
            step = (self._compute_stamp_ns - previous_stamp_ns) * 1e-9
            self._max_integration_step_seen = max(
                self._max_integration_step_seen, step
            )

        self._agents = response.updated_agents
        self._agents.header.frame_id = "map"
        self._agents.header.stamp = self._compute_stamp
        self._restore_static_agent_fields(self._agents)
        self._compute_count += 1
        self._compute_future = None
        self._compute_stamp = None
        self._compute_stamp_ns = None

        # HuNav state is authoritative. Isaac receives the newest completed
        # snapshot whenever its independent, lower-rate service is available.
        self._display_snapshot = copy.deepcopy(self._agents)
        self._needs_isaac_update = True
        self._agent_debug_pub.publish(self._agents)

    def _next_integration_stamp_ns(self):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns <= 0:
            return None
        if self._last_integrated_stamp_ns is None:
            return now_ns

        lag_ns = now_ns - self._last_integrated_stamp_ns
        if lag_ns < 0:
            self.get_logger().warning(
                "ROS simulation time moved backwards; re-seeding HuNav integration"
            )
            self._last_integrated_stamp_ns = None
            return now_ns
        if lag_ns < self._compute_period_ns:
            return None

        step_ns = min(lag_ns, self._max_integration_step_ns)
        if lag_ns > self._max_integration_step_ns:
            self._substep_count += 1
        return self._last_integrated_stamp_ns + step_ns

    def _start_hunav_compute(self):
        if self._compute_future is not None:
            return
        stamp_ns = self._next_integration_stamp_ns()
        if stamp_ns is None:
            return

        stamp = Time(nanoseconds=stamp_ns).to_msg()
        request = ComputeAgents.Request()
        request.current_agents = copy.deepcopy(self._agents)
        request.current_agents.header.frame_id = "map"
        request.current_agents.header.stamp = stamp
        request.robot = self._robot_agent()
        self._compute_stamp = stamp
        self._compute_stamp_ns = stamp_ns
        self._compute_future = self._compute.call_async(request)
        if self._metrics_start_monotonic is None:
            started_at = time.monotonic()
            self._metrics_start_monotonic = started_at
            self._last_report_monotonic = started_at
            self._last_reported_compute_count = self._compute_count
            self._last_reported_update_count = self._update_count

    def _start_isaac_update(self):
        if (
            self._update_future is not None
            or not self._needs_isaac_update
            or self._display_snapshot is None
        ):
            return

        now_monotonic = time.monotonic()
        if (
            now_monotonic - self._last_display_dispatch_monotonic
            < self._display_period
        ):
            return

        snapshot = self._display_snapshot
        request = UpdatePedestrians.Request()
        request.stamp = copy.deepcopy(snapshot.header.stamp)
        for agent in snapshot.agents:
            pedestrian = Pedestrian()
            pedestrian.name = agent.name
            pedestrian.pose = copy.deepcopy(agent.position)
            pedestrian.pose.position.z = 0.0
            pedestrian.twist = copy.deepcopy(agent.velocity)
            request.pedestrians.append(pedestrian)
        self._update_future = self._update_people.call_async(request)
        self._last_display_dispatch_monotonic = now_monotonic
        self._needs_isaac_update = False

    def _report_runtime(self):
        if (
            self._metrics_start_monotonic is None
            or self._last_report_monotonic is None
            or self._compute_count < self._status_interval
            or self._compute_count - self._last_reported_compute_count
            < self._status_interval
        ):
            return

        now_monotonic = time.monotonic()
        compute_count = self._compute_count
        update_count = self._update_count
        rates = calculate_runtime_rates(
            now_monotonic=now_monotonic,
            metrics_start_monotonic=self._metrics_start_monotonic,
            compute_count=compute_count,
            update_count=update_count,
            previous_report_monotonic=self._last_report_monotonic,
            previous_compute_count=self._last_reported_compute_count,
            previous_update_count=self._last_reported_update_count,
        )
        now_ns = self.get_clock().now().nanoseconds
        lag = max(0.0, (now_ns - self._last_integrated_stamp_ns) * 1e-9)
        states = ",".join(
            f"{agent.name}:{agent.behavior.type}/{agent.behavior.state}"
            for agent in sorted(self._agents.agents, key=lambda item: item.id)
        )
        self._status(
            format_runtime_status(
                compute_count=compute_count,
                update_count=update_count,
                rates=rates,
                max_integration_step=self._max_integration_step_seen,
                lag=lag,
                substep_count=self._substep_count,
                states=states,
            )
        )
        self._last_report_monotonic = now_monotonic
        self._last_reported_compute_count = compute_count
        self._last_reported_update_count = update_count

    def _tick(self):
        if self._odom is None:
            return

        # Neither slow display futures nor their rate limit can return early
        # from this scheduler. HuNav always progresses independently.
        self._finish_isaac_update()
        self._finish_hunav_compute()
        self._start_isaac_update()
        self._start_hunav_compute()
        self._report_runtime()

        if not self._ready_reported and self._update_count >= 10:
            types = sorted(agent.behavior.type for agent in self._agents.agents)
            ready_types = not self._strict_six_behavior_demo
            if self._strict_six_behavior_demo:
                ready_types = types == STRICT_SIX_BEHAVIOR_TYPES
            if ready_types:
                self._ready_reported = True
                self._status(
                    format_ready_status(
                        self._ready_marker,
                        self._agent_count,
                        types,
                        self._compute_rate,
                        self._max_integration_step_seen,
                    )
                )


def main(args=None):
    rclpy.init(args=args)
    node = ArenaHuNavIsaacBridge()
    try:
        node.spawn_scene()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        # rclpy can raise while its context is being invalidated by SIGINT.
        # That is normal shutdown, not a bridge failure.
        if rclpy.ok():
            node.get_logger().fatal(str(exc))
            raise
    finally:
        if rclpy.ok():
            try:
                node.destroy_node()
            except (KeyboardInterrupt, RuntimeError):
                pass
            if rclpy.ok():
                rclpy.shutdown()
