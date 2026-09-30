"""Run one HuNav state world without spawning or controlling any robot."""

from __future__ import annotations

import copy
import time

import rclpy
from arena_humble_compat.hunav_six_behaviors_bridge import ArenaHuNavIsaacBridge
from arena_people_msgs.msg import SpawnPedestrian
from arena_people_msgs.srv import SpawnPedestrians
from geometry_msgs.msg import Pose
from hunav_msgs.msg import Agent, Agents
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
import yaml


def _set_yaw(quaternion, yaw: float) -> None:
    import math

    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = math.sin(yaw / 2.0)
    quaternion.w = math.cos(yaw / 2.0)


class HuNavMultiAdapter(ArenaHuNavIsaacBridge):
    """Keep HuNav's one-robot API explicit: robot_1 is the only reaction source."""

    def __init__(self) -> None:
        super().__init__()
        self._agent_count = len(self._agents.agents)
        # The inherited single-robot bridge owns legacy global control edges.
        # This adapter only owns the pedestrian compute/display chain.
        for subscription in list(self.subscriptions):
            if subscription.topic_name in ("/odom", "/cmd_vel"):
                self.destroy_subscription(subscription)
        self.destroy_publisher(self._joint_pub)
        self.declare_parameter("status_topic", "/multirobot/hunav/status")
        self.declare_parameter("agent_debug_topic", "/multirobot/hunav/agents")
        self.declare_parameter("ready_marker", "MULTIROBOT_HUNAV_COMPUTE_READY")
        self.declare_parameter("strict_six_behavior_demo", False)
        self.destroy_publisher(self._status_pub)
        self.destroy_publisher(self._agent_debug_pub)
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self._agent_debug_pub = self.create_publisher(
            Agents, str(self.get_parameter("agent_debug_topic").value), 10
        )
        self._ready_marker = str(self.get_parameter("ready_marker").value)
        if bool(self.get_parameter("strict_six_behavior_demo").value):
            actual = sorted(agent.behavior.type for agent in self._agents.agents)
            if actual != [1, 2, 3, 4, 5, 6]:
                raise RuntimeError(f"six-behavior profile requires types 1..6, got {actual}")
        self.declare_parameter("reference_robot", "robot_1")
        self.reference_robot = str(self.get_parameter("reference_robot").value).strip("/")
        if self.reference_robot != "robot_1":
            raise RuntimeError("V1 HuNav reference_robot is fixed to robot_1")
        self._reference_odom_subscription = self.create_subscription(
            Odometry, f"/{self.reference_robot}/odom", self._on_odom, qos_profile_sensor_data
        )

    def _publish_wheels(self) -> None:
        """Disable the inherited wheel publisher; per-robot nodes own display joints."""

    def _status(self, text: str) -> None:
        """Keep the deliberately limited robot interaction scope observable."""
        if text.startswith("SIX_BEHAVIORS_RUNNING"):
            reference = getattr(self, "reference_robot", "robot_1")
            text += (
                f" reference={reference}"
                " interaction_scope=single_reference_robot"
            )
        super()._status(text)

    def _load_agents(self):
        """Load one or more agents without the legacy six-behavior restriction."""
        config_path = str(self.get_parameter("agent_config_file").value)
        if not config_path:
            raise RuntimeError("agent_config_file parameter is required")
        document = yaml.safe_load(open(config_path, encoding="utf-8"))
        try:
            parameters = document["hunav_loader"]["ros__parameters"]
            agent_names = parameters["agents"]
        except (KeyError, TypeError) as error:
            raise RuntimeError(f"invalid HuNav agent file: {config_path}") from error
        container = Agents()
        container.header.frame_id = "map"
        templates = {}
        for name in agent_names:
            spec = parameters[name]
            behavior = spec["behavior"]
            initial = spec["init_pose"]
            agent = Agent()
            agent.id = int(spec["id"])
            agent.type = Agent.PERSON
            agent.skin = int(spec["skin"])
            agent.name = str(name)
            agent.group_id = int(spec["group_id"])
            agent.position.position.x = float(initial["x"])
            agent.position.position.y = float(initial["y"])
            agent.yaw = float(initial["h"])
            _set_yaw(agent.position.orientation, agent.yaw)
            agent.desired_velocity = float(spec["max_vel"])
            agent.radius = float(spec["radius"])
            agent.behavior.type = int(behavior["type"])
            agent.behavior.state = 0
            agent.behavior.configuration = int(behavior["configuration"])
            agent.behavior.duration = float(behavior["duration"])
            agent.behavior.once = bool(behavior["once"])
            agent.behavior.vel = float(behavior["vel"])
            agent.behavior.dist = float(behavior["dist"])
            agent.behavior.goal_force_factor = float(behavior["goal_force_factor"])
            agent.behavior.obstacle_force_factor = float(behavior["obstacle_force_factor"])
            agent.behavior.social_force_factor = float(behavior["social_force_factor"])
            agent.behavior.other_force_factor = float(behavior["other_force_factor"])
            agent.goal_radius = float(spec["goal_radius"])
            agent.cyclic_goals = bool(spec["cyclic_goals"])
            for goal_name in spec["goals"]:
                goal_spec = spec[goal_name]
                goal = Pose()
                goal.position.x = float(goal_spec["x"])
                goal.position.y = float(goal_spec["y"])
                goal.orientation.w = 1.0
                agent.goals.append(goal)
            container.agents.append(agent)
            templates[agent.name] = copy.deepcopy(agent)
        if not container.agents:
            raise RuntimeError("HuNav profile must contain at least one agent")
        return container, templates

    def _finish_isaac_update(self):
        if self._update_future is None or not self._update_future.done():
            return
        if self._update_future.exception() is not None:
            self.get_logger().error(f"Isaac pedestrian update failed: {self._update_future.exception()}")
            self._needs_isaac_update = True
        else:
            result = self._update_future.result()
            if result and list(result.results) == [0] * self._agent_count:
                self._update_count += 1
            else:
                self.get_logger().error(f"Isaac pedestrian update returned errors: {result}")
                self._needs_isaac_update = True
        self._update_future = None

    def _finish_hunav_compute(self):
        if self._compute_future is None or not self._compute_future.done():
            return
        if self._compute_future.exception() is not None:
            self.get_logger().error(f"HuNav compute failed: {self._compute_future.exception()}")
            self._compute_future = None
            self._compute_stamp = None
            self._compute_stamp_ns = None
            return
        response = self._compute_future.result()
        if response is None or len(response.updated_agents.agents) != self._agent_count:
            self.get_logger().error(f"HuNav compute returned an invalid response: {response}")
            self._compute_future = None
            self._compute_stamp = None
            self._compute_stamp_ns = None
            return
        previous_stamp_ns = self._last_integrated_stamp_ns
        self._last_integrated_stamp_ns = self._compute_stamp_ns
        if previous_stamp_ns is not None:
            step = (self._compute_stamp_ns - previous_stamp_ns) * 1e-9
            self._max_integration_step_seen = max(self._max_integration_step_seen, step)
        self._agents = response.updated_agents
        self._agents.header.frame_id = "map"
        self._agents.header.stamp = self._compute_stamp
        self._restore_static_agent_fields(self._agents)
        self._compute_count += 1
        self._compute_future = None
        self._compute_stamp = None
        self._compute_stamp_ns = None
        self._display_snapshot = copy.deepcopy(self._agents)
        self._needs_isaac_update = True
        self._agent_debug_pub.publish(self._agents)

    def _tick(self):
        if self._odom is None:
            return
        self._finish_isaac_update()
        self._finish_hunav_compute()
        self._start_isaac_update()
        self._start_hunav_compute()
        self._report_runtime()
        if not self._ready_reported and self._update_count >= 10:
            self._ready_reported = True
            types = ",".join(str(agent.behavior.type) for agent in self._agents.agents)
            self._status(
                f"{self._ready_marker} pedestrians={self._agent_count} behavior_types={types} "
                f"reference={self.reference_robot} interaction_scope=single_reference_robot"
            )

    def _robot_agent(self):
        robot = super()._robot_agent()
        robot.name = self.reference_robot
        robot.type = Agent.ROBOT
        return robot

    def spawn_scene(self):
        deadline = time.monotonic() + 300.0
        while self._odom is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._odom is None:
            raise RuntimeError("timeout waiting for /robot_1/odom; robot scene is not ready")

        request = SpawnPedestrians.Request()
        for agent in self._agents.agents:
            item = SpawnPedestrian()
            item.model_ref = self._models[agent.name]
            item.pedestrian.name = agent.name
            item.pedestrian.pose = copy.deepcopy(agent.position)
            request.pedestrians.append(item)
        result = self._call(self._spawn_people, request, "/isaac/SpawnPedestrians", timeout=300.0)
        if not result or list(result.results) != [0] * self._agent_count:
            raise RuntimeError(f"pedestrian spawn failed: {result}")
        if not self._compute.wait_for_service(timeout_sec=60.0):
            raise RuntimeError("timeout waiting for HuNav /compute_agents")
        if not self._update_people.wait_for_service(timeout_sec=60.0):
            raise RuntimeError("timeout waiting for Isaac pedestrian updates")
        self._status(
            f"MULTIROBOT_HUNAV_READY reference={self.reference_robot} agents={self._agent_count} "
            "interaction_scope=single_reference_robot"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HuNavMultiAdapter()
    try:
        node.spawn_scene()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if rclpy.ok():
            node.get_logger().fatal(str(error))
            raise
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
