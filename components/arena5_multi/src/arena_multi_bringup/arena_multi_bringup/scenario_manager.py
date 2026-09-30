"""Create the shared walls and configured robots through Arena Isaac services."""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
from arena_multi_control.scenario import load_scenario
from isaacsim_msgs.msg import Wall
from isaacsim_msgs.srv import SpawnUrdf, SpawnWalls
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml


class ScenarioManager(Node):
    def __init__(self) -> None:
        super().__init__("scenario_manager")
        self.declare_parameter("scenario_file", "")
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("world_file", "")
        self.scenario_file = str(self.get_parameter("scenario_file").value)
        self.urdf_path = str(self.get_parameter("urdf_path").value)
        self.world_file = str(self.get_parameter("world_file").value)
        if not self.scenario_file or not self.urdf_path or not self.world_file:
            raise RuntimeError("scenario_file, urdf_path, and world_file are required")
        self.scenario = load_scenario(self.scenario_file)
        self.world = yaml.safe_load(Path(self.world_file).read_text(encoding="utf-8"))
        required = ("width", "height", "wall_height", "wall_thickness")
        if any(float(self.world.get(key, 0.0)) <= 0.0 for key in required):
            raise RuntimeError("world dimensions and wall dimensions must be positive")
        self.status = self.create_publisher(String, "/multirobot/status", 10)
        self.pause = self.create_client(Trigger, "/isaac/PauseSimulation")
        self.unpause = self.create_client(Trigger, "/isaac/UnpauseSimulation")
        self.walls = self.create_client(SpawnWalls, "/isaac/SpawnWalls")
        self.spawn = self.create_client(SpawnUrdf, "/isaac/SpawnUrdf")
        self.navigation_managers = {
            robot.name: self.create_client(
                ManageLifecycleNodes,
                f"/{robot.name}/lifecycle_manager_navigation/manage_nodes",
            )
            for robot in self.scenario.robots
            if robot.control_mode == "nav2"
        }

    def _call(self, client, request, name: str, timeout: float = 180.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"timeout waiting for {name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"timeout calling {name}")
        return future.result()

    def _publish(self, text: str) -> None:
        message = String()
        message.data = text
        self.status.publish(message)
        self.get_logger().info(text)

    def create_scene(self) -> None:
        paused = self._call(self.pause, Trigger.Request(), "/isaac/PauseSimulation")
        if not paused.success:
            raise RuntimeError("Isaac refused to pause")

        request = SpawnWalls.Request()
        width = float(self.world["width"])
        height = float(self.world["height"])
        wall_height = float(self.world["wall_height"])
        wall_thickness = float(self.world["wall_thickness"])
        corners = [((0.0, 0.0), (width, 0.0)), ((width, 0.0), (width, height)),
                   ((width, height), (0.0, height)), ((0.0, height), (0.0, 0.0))]
        for index, (start, end) in enumerate(corners):
            wall = Wall()
            wall.name = f"multirobot_boundary_{index}"
            wall.start.x, wall.start.y, wall.start.z = start[0], start[1], 0.0
            wall.end.x, wall.end.y, wall.end.z = end[0], end[1], wall_height
            wall.thickness = wall_thickness
            request.walls.append(wall)
        walls = self._call(self.walls, request, "/isaac/SpawnWalls")
        if not all(walls.ret):
            raise RuntimeError(f"wall creation failed: {list(walls.ret)}")

        for robot in self.scenario.robots:
            item = SpawnUrdf.Request()
            item.name = robot.name
            item.urdf_path = self.urdf_path
            item.robot_model = "jackal"
            item.tf_prefix = robot.name + "/"
            item.base_frame = "base_link"
            item.odom_frame = "odom"
            item.pose.position.x = robot.x
            item.pose.position.y = robot.y
            item.pose.position.z = 0.15
            item.pose.orientation.z = math.sin(robot.yaw / 2.0)
            item.pose.orientation.w = math.cos(robot.yaw / 2.0)
            item.cmd_vel_topic = f"/{robot.name}/cmd_vel"
            item.joint_states_topic = f"/{robot.name}/joint_states"
            item.odom_topic = f"/{robot.name}/odom"
            item.localization = True
            result = self._call(self.spawn, item, "/isaac/SpawnUrdf", timeout=300.0)
            if result.path != f"/World/{robot.name}":
                raise RuntimeError(f"unexpected robot path for {robot.name}: {result.path}")
            self._publish(f"MULTIROBOT_SPAWN_OK name={robot.name} path={result.path}")

        running = self._call(self.unpause, Trigger.Request(), "/isaac/UnpauseSimulation")
        if not running.success:
            raise RuntimeError("Isaac refused to unpause")
        names = ",".join(robot.name for robot in self.scenario.robots)
        self._publish(f"MULTIROBOT_SCENE_READY count={len(self.scenario.robots)} robots={names}")

        for name, client in self.navigation_managers.items():
            request = ManageLifecycleNodes.Request()
            request.command = ManageLifecycleNodes.Request.STARTUP
            result = self._call(
                client,
                request,
                f"/{name}/lifecycle_manager_navigation/manage_nodes",
                timeout=300.0,
            )
            if not result.success:
                raise RuntimeError(f"Nav2 lifecycle startup failed for {name}")
            self._publish(f"MULTIROBOT_NAV2_ACTIVE name={name}")

        self._publish(f"MULTIROBOT_READY count={len(self.scenario.robots)} robots={names}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScenarioManager()
    try:
        node.create_scene()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        node.get_logger().fatal(str(error))
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
