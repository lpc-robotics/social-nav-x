import math
import time

import rclpy
from arena_people_msgs.msg import Pedestrian, SpawnPedestrian
from arena_people_msgs.srv import SpawnPedestrians, UpdatePedestrians
from geometry_msgs.msg import Twist
from isaacsim_msgs.msg import Wall
from isaacsim_msgs.srv import SpawnUrdf, SpawnWalls
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


WHEEL_JOINTS = [
    "front_left_wheel_joint",
    "rear_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_right_wheel_joint",
]


class ArenaSceneBridge(Node):
    """Spawn one Arena scene and adapt Nav2 Twist commands to Isaac joints."""

    def __init__(self):
        super().__init__("arena_humble_compat")
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("dynamic_people", True)
        self.declare_parameter("robot_x", 3.0)
        self.declare_parameter("robot_y", 3.0)
        self.declare_parameter("robot_yaw", 0.0)

        self._joint_pub = self.create_publisher(
            JointState, "/isaac/joint_commands_velocity", 10
        )
        self._status_pub = self.create_publisher(String, "/arena5/status", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_twist, 10)
        self._last_twist = Twist()
        self._last_twist_monotonic = 0.0
        self.create_timer(0.05, self._publish_wheels)

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
        self._ped_start = time.monotonic()
        self._ped_future = None
        self._ped_updates = 0

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
            wall.name = f"arena_wall_{index}"
            wall.start.x, wall.start.y, wall.start.z = start[0], start[1], 0.0
            wall.end.x, wall.end.y, wall.end.z = end[0], end[1], 2.5
            wall.thickness = 0.1
            walls.walls.append(wall)
        wall_result = self._call(self._walls, walls, "/isaac/SpawnWalls")
        if not wall_result or not all(wall_result.ret):
            raise RuntimeError(f"wall spawn failed: {wall_result}")
        self._status("SCENE_WALLS_OK count=4 world=map_empty")

        dynamic_people = bool(self.get_parameter("dynamic_people").value)
        if dynamic_people:
            request = SpawnPedestrians.Request()
            item = SpawnPedestrian()
            item.model_ref = "original_female_adult_business_02"
            item.pedestrian.name = "arena_person_0"
            item.pedestrian.pose.position.x = 9.0
            item.pedestrian.pose.position.y = 6.0
            item.pedestrian.pose.orientation.w = 1.0
            request.pedestrians.append(item)
            result = self._call(
                self._spawn_people, request, "/isaac/SpawnPedestrians", timeout=300.0
            )
            if not result or list(result.results) != [0]:
                raise RuntimeError(f"pedestrian spawn failed: {result}")
            self._status(
                "SCENE_PEDESTRIAN_OK name=arena_person_0 model=original_female_adult_business_02"
            )

        # Spawn the physics articulation last. SpawnUrdf performs the single
        # World.reset() needed to rebuild the tensor view after stage edits.
        urdf_path = self.get_parameter("urdf_path").value
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
        robot_yaw = float(self.get_parameter("robot_yaw").value)
        robot.pose.orientation.z = math.sin(0.5 * robot_yaw)
        robot.pose.orientation.w = math.cos(0.5 * robot_yaw)
        robot.cmd_vel_topic = "/cmd_vel"
        robot.joint_states_topic = "/joint_states"
        robot.odom_topic = "/odom"
        robot.localization = True
        robot_result = self._call(self._robot, robot, "/isaac/SpawnUrdf")
        if not robot_result or not robot_result.path:
            raise RuntimeError(f"robot spawn failed: {robot_result}")
        self._status(f"SCENE_ROBOT_OK path={robot_result.path} model=jackal")

        unpause_result = self._call(
            self._unpause, Trigger.Request(), "/isaac/UnpauseSimulation"
        )
        if not unpause_result or not unpause_result.success:
            raise RuntimeError(f"could not unpause simulation: {unpause_result}")

        if dynamic_people:
            self._ped_start = time.monotonic()
            self.create_timer(0.1, self._update_pedestrian)

    def _on_twist(self, msg: Twist):
        self._last_twist = msg
        self._last_twist_monotonic = time.monotonic()

    def _publish_wheels(self):
        command = self._last_twist
        if time.monotonic() - self._last_twist_monotonic > 0.5:
            command = Twist()
        wheel_radius = 0.098
        wheel_separation = 0.37559
        left = (command.linear.x - command.angular.z * wheel_separation / 2.0) / wheel_radius
        right = (command.linear.x + command.angular.z * wheel_separation / 2.0) / wheel_radius
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = WHEEL_JOINTS
        msg.velocity = [left, left, right, right]
        self._joint_pub.publish(msg)

    def _update_pedestrian(self):
        if self._ped_future is not None:
            if not self._ped_future.done():
                return
            if self._ped_future.exception() is not None:
                self.get_logger().error(
                    f"pedestrian update failed: {self._ped_future.exception()}"
                )
            else:
                result = self._ped_future.result()
                if result and list(result.results) == [0]:
                    self._ped_updates += 1
                    if self._ped_updates == 10:
                        self._status("SCENE_PEDESTRIAN_DYNAMIC_OK updates=10")
            self._ped_future = None

        elapsed = time.monotonic() - self._ped_start
        omega = 0.35
        amplitude = 2.0
        pedestrian = Pedestrian()
        pedestrian.name = "arena_person_0"
        pedestrian.pose.position.x = 9.0
        pedestrian.pose.position.y = 6.0 + amplitude * math.sin(omega * elapsed)
        pedestrian.pose.orientation.w = 1.0
        pedestrian.twist.linear.y = amplitude * omega * math.cos(omega * elapsed)
        request = UpdatePedestrians.Request()
        request.pedestrians.append(pedestrian)
        self._ped_future = self._update_people.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = ArenaSceneBridge()
    try:
        node.spawn_scene()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.get_logger().fatal(str(exc))
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()
