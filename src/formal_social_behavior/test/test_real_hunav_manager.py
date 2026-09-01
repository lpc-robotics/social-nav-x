import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time
import unittest
import uuid

import rclpy
from geometry_msgs.msg import Pose
from hunav_msgs.msg import Agent, AgentBehavior
from hunav_msgs.srv import ComputeAgents
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from formal_social_behavior.model import FormalState
from formal_social_behavior.proxy_node import FormalSocialBehaviorProxy


def _wait_for_future(future, timeout=10.0):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.005)
    if not future.done():
        raise AssertionError("ROS service future did not complete")
    return future.result()


class RealHuNavManagerServiceTest(unittest.TestCase):
    """Exercise the proxy against HuNav's real C++ manager without Isaac."""

    def setUp(self):
        rclpy.init()
        self.suffix = uuid.uuid4().hex[:8]
        service_root = f"/formal_real_hunav_{self.suffix}"
        self.compute_service = service_root + "/compute"
        self.raw_compute_service = service_root + "/compute_raw"
        self.raw_reset_service = service_root + "/reset_raw"
        self.tempdir = tempfile.TemporaryDirectory()
        self.manager_log_path = Path(self.tempdir.name) / "hunav_manager.log"
        self.manager_log = self.manager_log_path.open("w", encoding="utf-8")
        config_path = (
            Path(__file__).resolve().parents[1]
            / "config"
            / "formal_social_automata.yaml"
        )

        command = [
            "ros2",
            "run",
            "hunav_agent_manager",
            "hunav_agent_manager",
            "--ros-args",
            "-r",
            f"__node:=real_hunav_manager_{self.suffix}",
            "-r",
            f"compute_agents:={self.raw_compute_service}",
            "-r",
            f"reset_agents:={self.raw_reset_service}",
            "-p",
            "publish_tf:=false",
            "-p",
            "publish_sfm_forces:=false",
            "-p",
            "hunav_loader.publish_people:=false",
        ]
        self.manager = subprocess.Popen(
            command,
            env=os.environ.copy(),
            stdout=self.manager_log,
            stderr=subprocess.STDOUT,
        )
        self.proxy = FormalSocialBehaviorProxy(
            node_name=f"formal_real_proxy_{self.suffix}",
            parameter_overrides=[
                Parameter("enabled", value=True),
                Parameter("config_file", value=str(config_path)),
                Parameter("compute_service", value=self.compute_service),
                Parameter(
                    "raw_compute_service", value=self.raw_compute_service
                ),
                Parameter("raw_reset_service", value=self.raw_reset_service),
                Parameter("service_timeout_seconds", value=5.0),
            ],
        )
        self.client_node = Node(f"formal_real_client_{self.suffix}")
        self.client = self.client_node.create_client(
            ComputeAgents, self.compute_service
        )
        self.executor = MultiThreadedExecutor(num_threads=6)
        self.executor.add_node(self.proxy)
        self.executor.add_node(self.client_node)
        self.spin_thread = threading.Thread(
            target=self.executor.spin, daemon=True
        )
        self.spin_thread.start()
        self.assertTrue(self.client.wait_for_service(timeout_sec=3.0))
        if not self.proxy._raw_compute.wait_for_service(timeout_sec=10.0):
            self.fail("real HuNav compute service did not start: " + self._log())
        if not self.proxy._raw_reset.wait_for_service(timeout_sec=3.0):
            self.fail("real HuNav reset service did not start: " + self._log())

    def tearDown(self):
        if self.manager.poll() is None:
            self.manager.terminate()
            try:
                self.manager.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.manager.kill()
                self.manager.wait(timeout=5.0)
        self.executor.shutdown(timeout_sec=3.0)
        self.spin_thread.join(timeout=3.0)
        for node in (self.client_node, self.proxy):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.manager_log.close()
        self.tempdir.cleanup()

    def _log(self):
        self.manager_log.flush()
        return self.manager_log_path.read_text(
            encoding="utf-8", errors="replace"
        )[-4000:]

    @staticmethod
    def _request(stamp_seconds, robot_x, robot_vx=0.0):
        request = ComputeAgents.Request()
        seconds = int(stamp_seconds)
        nanoseconds = int(round((stamp_seconds - seconds) * 1.0e9))
        request.current_agents.header.stamp.sec = seconds
        request.current_agents.header.stamp.nanosec = nanoseconds
        request.current_agents.header.frame_id = "map"

        human = Agent()
        human.id = 1
        human.type = Agent.PERSON
        human.name = "formal_human"
        human.position.orientation.w = 1.0
        human.yaw = 0.0
        human.desired_velocity = 0.6
        human.radius = 0.4
        human.behavior.type = AgentBehavior.BEH_REGULAR
        human.behavior.state = 0
        human.behavior.configuration = AgentBehavior.BEH_CONF_CUSTOM
        human.behavior.duration = 40.0
        human.behavior.once = True
        human.behavior.vel = 0.6
        human.behavior.dist = 0.0
        human.behavior.goal_force_factor = 2.0
        human.behavior.obstacle_force_factor = 10.0
        human.behavior.social_force_factor = 5.0
        human.behavior.other_force_factor = 20.0
        human.goals = [Pose()]
        human.goals[0].position.x = 6.0
        human.goals[0].position.y = 9.0
        human.goals[0].orientation.w = 1.0
        human.cyclic_goals = True
        human.goal_radius = 0.3
        request.current_agents.agents = [human]

        request.robot.id = 0
        request.robot.type = Agent.ROBOT
        request.robot.name = "jackal"
        request.robot.position.position.x = float(robot_x)
        request.robot.position.orientation.w = 1.0
        request.robot.velocity.linear.x = float(robot_vx)
        request.robot.radius = 0.35
        return request

    def _compute(self, stamp_seconds, robot_x, robot_vx=0.0):
        future = self.client.call_async(
            self._request(stamp_seconds, robot_x, robot_vx)
        )
        return _wait_for_future(future)

    def test_real_manager_rebuilds_each_changed_profile_once(self):
        observations = (
            (0.0, 5.0, 0.0, FormalState.ATTENTION, 1),
            (0.5, 2.5, -0.15, FormalState.CURIOUS, 5),
            (0.6, 2.5, -0.15, FormalState.CURIOUS, 5),
            (1.0, 2.0, -0.8, FormalState.SCARED, 4),
            (2.0, 3.0, 0.2, FormalState.SCARED, 4),
            (5.0, 3.6, 0.2, FormalState.NORMAL, 1),
            (6.0, 3.0, -0.3, FormalState.ATTENTION, 1),
            (6.1, 2.5, -0.3, FormalState.SURPRISED, 3),
            (7.0, 7.0, 0.0, FormalState.NORMAL, 1),
        )
        returned_types = []
        for stamp, x, vx, expected_state, expected_type in observations:
            response = self._compute(stamp, x, vx)
            if len(response.updated_agents.agents) != 1:
                self.fail("real HuNav returned an invalid response: " + self._log())
            returned = response.updated_agents.agents[0]
            returned_types.append(int(returned.behavior.type))
            self.assertEqual(returned.id, 1)
            self.assertEqual(returned.name, "formal_human")
            self.assertEqual(len(returned.goals), 1)
            self.assertEqual(int(returned.behavior.type), expected_type)
            self.assertEqual(
                self.proxy._automaton_context.state, expected_state
            )

        self.assertEqual(returned_types, [1, 5, 5, 4, 4, 1, 1, 3, 1])
        self.assertEqual(self.proxy._reset_count, 5)
        self.manager_log.flush()
        manager_log = self.manager_log_path.read_text(
            encoding="utf-8", errors="replace"
        )
        self.assertEqual(
            manager_log.count("=== RESET AGENTS SERVICE CALLED ==="),
            5,
            manager_log,
        )
        self.assertIsNone(self.manager.poll(), self._log())


if __name__ == "__main__":
    unittest.main()
