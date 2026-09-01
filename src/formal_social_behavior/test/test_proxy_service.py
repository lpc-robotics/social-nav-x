import copy
import json
import math
from pathlib import Path
import tempfile
import threading
import time
import unittest
import uuid

import rclpy
from geometry_msgs.msg import Pose
from hunav_msgs.msg import Agent, AgentBehavior
from hunav_msgs.srv import ComputeAgents, ResetAgents
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from formal_social_behavior.model import FormalState
from formal_social_behavior.proxy_node import (
    FormalSocialBehaviorProxy,
    _service_timeout_seconds,
)


def _wait_for_future(future, timeout=5.0):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.005)
    if not future.done():
        raise AssertionError("ROS service future did not complete")
    return future.result()


class FakeRawHuNav(Node):
    def __init__(self, suffix, compute_service, reset_service):
        super().__init__(f"fake_raw_hunav_{suffix}")
        self.compute_types = []
        self.reset_types = []
        self.reset_ok = True
        self.fail_compute_once = False
        self.compute_delay_seconds = 0.0
        self.reset_delay_seconds = 0.0
        self.operation_events = []
        self.create_service(
            ComputeAgents, compute_service, self._compute
        )
        self.create_service(ResetAgents, reset_service, self._reset)

    def _compute(self, request, response):
        behavior_type = int(request.current_agents.agents[0].behavior.type)
        self.operation_events.append(f"compute:{behavior_type}")
        if self.compute_delay_seconds > 0.0:
            time.sleep(self.compute_delay_seconds)
        self.compute_types.append(behavior_type)
        if self.fail_compute_once:
            self.fail_compute_once = False
            return response
        response.updated_agents = copy.deepcopy(request.current_agents)
        return response

    def _reset(self, request, response):
        behavior_type = int(request.current_agents.agents[0].behavior.type)
        self.operation_events.append(f"reset:{behavior_type}")
        if self.reset_delay_seconds > 0.0:
            time.sleep(self.reset_delay_seconds)
        self.reset_types.append(behavior_type)
        response.ok = self.reset_ok
        return response


class ProxyParameterValidationTests(unittest.TestCase):
    def test_service_timeout_requires_positive_finite_real_number(self):
        self.assertEqual(_service_timeout_seconds(2), 2.0)
        self.assertEqual(_service_timeout_seconds(0.25), 0.25)
        for invalid_value in (
            True,
            "1.0",
            0,
            -1,
            math.nan,
            math.inf,
            -math.inf,
        ):
            with self.subTest(value=invalid_value):
                with self.assertRaises(RuntimeError):
                    _service_timeout_seconds(invalid_value)


class ProxyServiceTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.suffix = uuid.uuid4().hex[:8]
        service_root = f"/formal_social_test_{self.suffix}"
        self.compute_service = service_root + "/compute"
        self.raw_compute_service = service_root + "/compute_raw"
        self.raw_reset_service = service_root + "/reset_raw"
        self.tempdir = tempfile.TemporaryDirectory()
        self.trace_path = Path(self.tempdir.name) / "transitions.jsonl"
        config_path = (
            Path(__file__).resolve().parents[1]
            / "config"
            / "formal_social_automata.yaml"
        )

        self.raw = FakeRawHuNav(
            self.suffix,
            self.raw_compute_service,
            self.raw_reset_service,
        )
        self.proxy = FormalSocialBehaviorProxy(
            node_name=f"formal_social_proxy_{self.suffix}",
            parameter_overrides=[
                Parameter("enabled", value=True),
                Parameter("config_file", value=str(config_path)),
                Parameter("trace_file", value=str(self.trace_path)),
                Parameter("compute_service", value=self.compute_service),
                Parameter(
                    "raw_compute_service", value=self.raw_compute_service
                ),
                Parameter("raw_reset_service", value=self.raw_reset_service),
                Parameter("service_timeout_seconds", value=2.0),
            ],
        )
        self.client_node = Node(f"formal_social_client_{self.suffix}")
        self.client = self.client_node.create_client(
            ComputeAgents, self.compute_service
        )
        self.executor = MultiThreadedExecutor(num_threads=4)
        for node in (self.raw, self.proxy, self.client_node):
            self.executor.add_node(node)
        self.spin_thread = threading.Thread(
            target=self.executor.spin, daemon=True
        )
        self.spin_thread.start()
        self.assertTrue(self.client.wait_for_service(timeout_sec=3.0))

    def tearDown(self):
        self.executor.shutdown(timeout_sec=3.0)
        self.spin_thread.join(timeout=3.0)
        for node in (self.client_node, self.proxy, self.raw):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.tempdir.cleanup()

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
        human.goals[0].orientation.w = 1.0
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

    def test_profiles_reset_once_and_follow_all_v1_paths(self):
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
        for stamp, x, vx, state, behavior_type in observations:
            response = self._compute(stamp, x, vx)
            self.assertEqual(len(response.updated_agents.agents), 1)
            returned = response.updated_agents.agents[0]
            returned_types.append(int(returned.behavior.type))
            self.assertEqual(returned.id, 1)
            self.assertEqual(returned.name, "formal_human")
            self.assertEqual(self.proxy._automaton_context.state, state)
            self.assertEqual(int(returned.behavior.type), behavior_type)

        self.assertEqual(returned_types, [1, 5, 5, 4, 4, 1, 1, 3, 1])
        self.assertEqual(self.raw.compute_types, returned_types)
        self.assertEqual(self.raw.reset_types, [5, 4, 1, 3, 1])
        self.assertEqual(self.proxy._reset_count, 5)

    def test_first_non_regular_profile_resets_from_request_profile(self):
        response = self._compute(0.0, 2.5, -0.30)

        self.assertEqual(len(response.updated_agents.agents), 1)
        self.assertEqual(
            self.proxy._automaton_context.state, FormalState.SURPRISED
        )
        self.assertEqual(self.raw.reset_types, [3])
        self.assertEqual(self.raw.compute_types, [3])
        self.assertEqual(self.proxy._reset_count, 1)

    def test_reset_and_compute_failures_do_not_commit_candidates(self):
        initial = self._compute(0.0, 5.0)
        self.assertEqual(len(initial.updated_agents.agents), 1)
        self.assertEqual(
            self.proxy._automaton_context.state, FormalState.ATTENTION
        )

        self.raw.reset_ok = False
        rejected_reset = self._compute(0.5, 2.5, -0.15)
        self.assertEqual(len(rejected_reset.updated_agents.agents), 0)
        self.assertEqual(
            self.proxy._automaton_context.state, FormalState.ATTENTION
        )
        self.raw.reset_ok = True
        accepted_retry = self._compute(0.5, 2.5, -0.15)
        self.assertEqual(len(accepted_retry.updated_agents.agents), 1)
        self.assertEqual(
            self.proxy._automaton_context.state, FormalState.CURIOUS
        )

        self.raw.fail_compute_once = True
        rejected_compute = self._compute(1.0, 2.0, -0.8)
        self.assertEqual(len(rejected_compute.updated_agents.agents), 0)
        self.assertEqual(
            self.proxy._automaton_context.state, FormalState.CURIOUS
        )
        accepted_compute_retry = self._compute(1.0, 2.0, -0.8)
        self.assertEqual(len(accepted_compute_retry.updated_agents.agents), 1)
        self.assertEqual(
            self.proxy._automaton_context.state, FormalState.SCARED
        )

        self.assertEqual(self.raw.reset_types, [5, 5, 4, 4])
        self.assertEqual(self.raw.compute_types, [1, 5, 4, 4])
        trace = [
            json.loads(line)
            for line in self.trace_path.read_text(encoding="utf-8").splitlines()
        ]
        self.assertEqual(
            [record["new_state"] for record in trace],
            ["ATTENTION", "CURIOUS", "SCARED"],
        )

    def test_disabled_mode_only_passes_through(self):
        self.executor.remove_node(self.proxy)
        self.proxy.destroy_node()
        disabled_service = self.compute_service + "_disabled"
        self.proxy = FormalSocialBehaviorProxy(
            node_name=f"formal_disabled_proxy_{self.suffix}",
            parameter_overrides=[
                Parameter("enabled", value=False),
                Parameter("compute_service", value=disabled_service),
                Parameter(
                    "raw_compute_service", value=self.raw_compute_service
                ),
                Parameter("raw_reset_service", value=self.raw_reset_service),
                Parameter("service_timeout_seconds", value=2.0),
            ],
        )
        self.executor.add_node(self.proxy)
        self.client = self.client_node.create_client(
            ComputeAgents, disabled_service
        )
        self.assertTrue(self.client.wait_for_service(timeout_sec=3.0))

        future = self.client.call_async(self._request(0.0, 5.0))
        response = _wait_for_future(future)
        self.assertEqual(len(response.updated_agents.agents), 1)
        self.assertEqual(response.updated_agents.agents[0].behavior.type, 1)
        self.assertEqual(self.raw.compute_types, [1])
        self.assertEqual(self.raw.reset_types, [])
        self.assertEqual(
            self.proxy._automaton_context.state, FormalState.NORMAL
        )
        self.assertEqual(self.proxy._reset_count, 0)

    def test_concurrent_requests_keep_each_reset_compute_pair_serial(self):
        initial = self._compute(0.0, 5.0)
        self.assertEqual(len(initial.updated_agents.agents), 1)
        self.raw.operation_events.clear()
        self.raw.reset_delay_seconds = 0.05

        first = self.client.call_async(self._request(0.5, 2.5, -0.15))
        second = self.client.call_async(self._request(1.0, 2.0, -0.8))
        responses = (_wait_for_future(first), _wait_for_future(second))

        self.assertTrue(
            all(len(response.updated_agents.agents) == 1 for response in responses)
        )
        self.assertEqual(len(self.raw.operation_events), 4)
        self.assertTrue(self.raw.operation_events[0].startswith("reset:"))
        self.assertTrue(self.raw.operation_events[1].startswith("compute:"))
        self.assertTrue(self.raw.operation_events[2].startswith("reset:"))
        self.assertTrue(self.raw.operation_events[3].startswith("compute:"))

    def test_four_concurrent_requests_do_not_starve_raw_future_callbacks(self):
        initial = self._compute(0.0, 5.0)
        self.assertEqual(len(initial.updated_agents.agents), 1)
        self.raw.compute_types.clear()
        self.raw.compute_delay_seconds = 0.05

        futures = [
            self.client.call_async(self._request(0.1, 5.0))
            for _ in range(4)
        ]
        responses = [_wait_for_future(future) for future in futures]

        self.assertTrue(
            all(len(response.updated_agents.agents) == 1 for response in responses)
        )
        self.assertEqual(self.raw.compute_types, [1, 1, 1, 1])
        self.assertEqual(self.raw.reset_types, [])

    def test_compute_timeout_does_not_commit_candidate(self):
        initial = self._compute(0.0, 5.0)
        self.assertEqual(len(initial.updated_agents.agents), 1)
        self.assertEqual(
            self.proxy._automaton_context.state, FormalState.ATTENTION
        )

        self.proxy._timeout = 0.05
        self.raw.compute_delay_seconds = 0.20
        response = self._compute(0.5, 5.0)
        self.assertEqual(len(response.updated_agents.agents), 0)
        self.assertEqual(
            self.proxy._automaton_context.state, FormalState.ATTENTION
        )
        self.assertEqual(self.proxy._reset_count, 0)
        time.sleep(0.20)

    def test_telemetry_failure_does_not_retry_successful_raw_compute(self):
        def fail_telemetry(*_args, **_kwargs):
            raise RuntimeError("injected telemetry failure")

        self.proxy._publish_observation = fail_telemetry
        response = self._compute(0.0, 5.0)

        self.assertEqual(len(response.updated_agents.agents), 1)
        self.assertEqual(self.raw.compute_types, [1])
        self.assertEqual(self.raw.reset_types, [])
        self.assertEqual(
            self.proxy._automaton_context.state, FormalState.ATTENTION
        )

    def test_clock_rollback_regular_to_regular_does_not_reset(self):
        initial = self._compute(1.0, 5.0)
        self.assertEqual(len(initial.updated_agents.agents), 1)
        self.assertEqual(
            self.proxy._automaton_context.state, FormalState.ATTENTION
        )

        rollback = self._compute(0.5, 5.0)
        self.assertEqual(len(rollback.updated_agents.agents), 1)
        self.assertEqual(
            self.proxy._automaton_context.state, FormalState.NORMAL
        )
        self.assertEqual(self.raw.compute_types, [1, 1])
        self.assertEqual(self.raw.reset_types, [])
        self.assertEqual(self.proxy._reset_count, 0)

    def test_clock_rollback_resets_when_profile_tuple_changes(self):
        self.assertEqual(len(self._compute(1.0, 5.0).updated_agents.agents), 1)
        curious = self._compute(1.5, 2.5, -0.15)
        self.assertEqual(len(curious.updated_agents.agents), 1)
        self.assertEqual(
            self.proxy._automaton_context.state, FormalState.CURIOUS
        )

        rollback = self._compute(1.25, 2.5, -0.15)
        self.assertEqual(len(rollback.updated_agents.agents), 1)
        self.assertEqual(
            self.proxy._automaton_context.state, FormalState.NORMAL
        )
        self.assertEqual(self.raw.compute_types, [1, 5, 1])
        self.assertEqual(self.raw.reset_types, [5, 1])
        self.assertEqual(self.proxy._reset_count, 2)


if __name__ == "__main__":
    unittest.main()
