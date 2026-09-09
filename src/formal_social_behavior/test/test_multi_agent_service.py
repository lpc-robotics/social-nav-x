"""Batch atomicity, actual ROS concurrency, and late-result fencing."""

import copy
import json
import math
import threading
import time
import uuid
from pathlib import Path

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Pose
from hunav_msgs.msg import Agent
from hunav_msgs.srv import ComputeAgents, ResetAgents

from formal_social_behavior.behavior_adapter import apply_profile_to_agent, default_behavior_profiles
from formal_social_behavior.model import FormalState
from formal_social_behavior.multi_agent.proxy_node import MultiAgentProxy, request_fingerprint
from formal_social_behavior.multi_agent.replay import verify_replay
from test_multi_agent_core import scene


CONFIG = Path(__file__).resolve().parents[1] / "config/formal_social_multi_automata.yaml"


def request_for(snapshot):
    request = ComputeAgents.Request()
    request.current_agents.header.frame_id = "map"
    request.current_agents.header.stamp.sec = snapshot.sim_time_ns // 1_000_000_000
    request.current_agents.header.stamp.nanosec = snapshot.sim_time_ns % 1_000_000_000
    for agent_id, kinematics in ((0, snapshot.robot), *snapshot.humans):
        agent = Agent()
        agent.id = agent_id
        agent.type = Agent.ROBOT if agent_id == 0 else Agent.PERSON
        agent.name = "robot" if agent_id == 0 else f"human_{agent_id}"
        agent.group_id = -1
        agent.radius = kinematics.radius
        agent.desired_velocity = .6
        agent.position.position.x = float(kinematics.x)
        agent.position.position.y = float(kinematics.y)
        agent.position.orientation.z = math.sin(kinematics.yaw / 2)
        agent.position.orientation.w = math.cos(kinematics.yaw / 2)
        agent.yaw = kinematics.yaw
        agent.velocity.linear.x = float(kinematics.vx)
        agent.velocity.linear.y = float(kinematics.vy)
        agent.linear_vel = math.hypot(kinematics.vx, kinematics.vy)
        agent = apply_profile_to_agent(agent, default_behavior_profiles()[FormalState.NORMAL])
        agent.goal_radius = .3
        agent.cyclic_goals = True
        if agent_id:
            goal = Pose()
            goal.position.x, goal.position.y = float(kinematics.x), float(kinematics.y)
            goal.orientation.w = 1.
            agent.goals = [goal]
            request.current_agents.agents.append(agent)
        else:
            request.robot = agent
    return request


def wait(future, timeout=10):
    complete = threading.Event()
    future.add_done_callback(lambda _: complete.set())
    assert complete.wait(timeout), "ROS future did not complete"
    return future.result()


class Harness:
    def __init__(self, tmp_path, *, enabled=True, shared=True, timeout=.5, raw_root=None):
        rclpy.init()
        root = "/multi_test_" + uuid.uuid4().hex[:10]
        self.root = raw_root or root
        self.calls = []
        self.fail_reset = False
        self.fault = None
        self.delay = 0
        self.reset_delay = 0
        self.backend = Node("backend_" + root[1:])
        if raw_root is None:
            self.backend.create_service(ComputeAgents, root + "/raw_compute", self.compute)
            self.backend.create_service(ResetAgents, root + "/raw_reset", self.reset)
        parameters = {"enabled": enabled, "shared_events_enabled": shared,
                      "config_file": str(CONFIG), "trace_file": str(tmp_path / "transitions.jsonl"),
                      "steps_file": str(tmp_path / "steps.jsonl"), "compute_service": root + "/compute",
                      "raw_compute_service": self.root + "/raw_compute",
                      "raw_reset_service": self.root + "/raw_reset", "topic_prefix": root + "/topics",
                      "service_timeout_seconds": float(timeout)}
        self.proxy = MultiAgentProxy(node_name="proxy_" + root[1:], parameter_overrides=[
            Parameter(key, value=value) for key, value in parameters.items()])
        # Observe proxy rejection while the deliberately slow raw service is
        # still running; sharing its exclusive group would hide quarantine.
        self.client = self.backend.create_client(ComputeAgents, root + "/compute",
                                                callback_group=ReentrantCallbackGroup())
        self.executor = MultiThreadedExecutor(num_threads=6)
        self.executor.add_node(self.backend)
        self.executor.add_node(self.proxy)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        assert self.client.wait_for_service(timeout_sec=3)
        assert self.proxy.raw_compute.wait_for_service(timeout_sec=10)
        self.tmp_path = tmp_path

    def close(self):
        self.proxy.request_shutdown()
        self.executor.shutdown(timeout_sec=3)
        self.thread.join(3)
        self.proxy.destroy_node()
        self.backend.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def reset(self, request, response):
        self.calls.append(("reset", copy.deepcopy(request)))
        if self.reset_delay:
            time.sleep(self.reset_delay)
        response.ok = not self.fail_reset
        return response

    def compute(self, request, response):
        self.calls.append(("compute", copy.deepcopy(request)))
        if self.delay:
            time.sleep(self.delay)
        if self.fault == "empty":
            return response
        response.updated_agents = copy.deepcopy(request.current_agents)
        response.updated_agents.agents = list(reversed(response.updated_agents.agents))
        agents = response.updated_agents.agents
        if self.fault == "duplicate":
            agents[0].id = agents[1].id
        elif self.fault == "nan":
            agents[0].velocity.linear.x = float("nan")
        elif self.fault == "type":
            agents[0].behavior.type = 6
        elif self.fault == "name":
            agents[0].name = "wrong"
        elif self.fault == "goals":
            agents[0].goals = []
        elif self.fault == "profile_nan":
            agents[0].behavior.vel = float("nan")
        elif self.fault == "static_omitted":
            for agent in agents:
                agent.radius = agent.goal_radius = agent.desired_velocity = 0.
                agent.group_id = 0
                agent.cyclic_goals = False
        return response

    def call(self, snapshot):
        return wait(self.client.call_async(request_for(snapshot)))

    def form(self):
        self.call(scene(0))
        self.call(scene(1))
        assert self.states() == ("SOCIAL", "SOCIAL")

    def states(self):
        return tuple(agent.context.state.value for agent in self.proxy.committed.agents)

    def frames(self):
        return [json.loads(line) for line in (self.tmp_path / "steps.jsonl").read_text().splitlines()]


@pytest.fixture
def harness(tmp_path):
    item = Harness(tmp_path)
    try:
        yield item
    finally:
        item.close()


def test_batch_profile_change_reorders_ids_and_preserves_local_clocks(harness):
    h = harness
    h.form()
    assert h.proxy.reset_count == 0
    result = h.call(scene(2, 5.21, vx=.15))
    assert [agent.id for agent in result.updated_agents.agents] == [1, 2]
    assert h.states() == ("SURPRISED", "SURPRISED")
    assert h.proxy.reset_count == 1
    entered_b = h.proxy.committed.agents[1].context.state_entered_ns
    h.call(scene(2.1, 5.21, 2.4, .15))
    assert h.states() == ("SCARED", "SURPRISED")
    assert h.proxy.committed.agents[1].context.state_entered_ns == entered_b
    assert h.frames()[-1]["changed_profile_ids"] == [1]
    assert h.frames()[-1]["reset_affected_ids"] == [1, 2]
    assert [kind for kind, _ in h.calls] == ["compute", "compute", "reset", "compute", "reset", "compute"]
    lines = (h.tmp_path / "steps.jsonl").read_text().splitlines()
    assert verify_replay(CONFIG.read_bytes(), lines) == 4
    damaged = json.loads(lines[-1])
    damaged["agents"][0]["state"] = "NORMAL"
    with pytest.raises(ValueError, match="semantic mismatch"):
        verify_replay(CONFIG.read_bytes(), [*lines[:-1], json.dumps(damaged)])


@pytest.mark.parametrize("fault", ["empty", "duplicate", "nan", "type", "name", "goals", "profile_nan"])
def test_invalid_raw_batch_rolls_back_every_context(harness, fault):
    h = harness
    h.form()
    committed = h.proxy.committed
    h.fault = fault
    result = h.call(scene(2, 5.21))
    assert not result.updated_agents.agents
    assert h.proxy.committed is committed and h.proxy.reset_count == 0
    assert len(h.frames()) == 2 and h.proxy.backend_dirty
    h.fault = None
    h.call(scene(2, 5.21))
    assert h.states() == ("SURPRISED", "SURPRISED")
    assert h.proxy.reset_attempts == 2 and h.proxy.reset_count == 1
    assert h.frames()[-1]["reset_reason"] == "resynchronize"


def test_reset_rejection_and_same_profile_failure_force_resynchronization(harness):
    h = harness
    h.form()
    h.fail_reset = True
    h.call(scene(2, 5.21))
    assert h.states() == ("SOCIAL", "SOCIAL")
    assert h.calls[-1][0] == "reset"
    h.fail_reset = False
    h.call(scene(2.1))  # Profile itself has not changed, but backend is dirty.
    assert h.proxy.reset_count == 1
    assert h.frames()[-1]["changed_profile_ids"] == []


def test_static_fields_and_caller_request_preserved(harness):
    h = harness
    h.form()
    h.fault = "static_omitted"
    request = request_for(scene(2))
    before = copy.deepcopy(request)
    response = wait(h.client.call_async(request))
    assert request == before
    for agent, original in zip(response.updated_agents.agents, request.current_agents.agents):
        assert agent.radius == pytest.approx(original.radius, abs=1e-7)
        assert agent.goal_radius == pytest.approx(original.goal_radius, abs=1e-7)
        assert agent.group_id == -1 and agent.cyclic_goals


def test_duplicate_is_cached_but_conflicting_stamp_and_invalid_input_are_rejected(harness):
    h = harness
    h.form()
    count = len(h.calls)
    request = request_for(scene(1))
    request.current_agents.agents.reverse()
    response = wait(h.client.call_async(request))
    assert [agent.id for agent in response.updated_agents.agents] == [2, 1]
    assert len(h.calls) == count and len(h.frames()) == 2
    assert not h.call(scene(1, 4)).updated_agents.agents
    request = request_for(scene(2))
    request.current_agents.agents[1].id = 1
    assert not wait(h.client.call_async(request)).updated_agents.agents
    assert len(h.calls) == count and not h.proxy.backend_dirty


def test_request_fingerprint_uses_all_canonical_fields():
    request = request_for(scene(2, 5.21))
    fingerprint = request_fingerprint(request)
    assert all(request_fingerprint(request_for(scene(2, 5.21))) == fingerprint for _ in range(100))
    changed = copy.deepcopy(request)
    changed.current_agents.agents[0].goals[0].position.x += .1
    assert request_fingerprint(changed) != fingerprint
    changed = copy.deepcopy(request)
    changed.current_agents.agents[1].radius += .1
    assert request_fingerprint(changed) != fingerprint


def test_four_concurrent_duplicates_do_not_interleave_or_starve(harness):
    h = harness
    h.form()
    h.delay = .05
    futures = [h.client.call_async(request_for(scene(2, 5.21))) for _ in range(4)]
    assert all(len(wait(future).updated_agents.agents) == 2 for future in futures)
    assert [kind for kind, _ in h.calls][-2:] == ["reset", "compute"]
    assert len(h.calls) == 4 and h.proxy.reset_count == 1 and len(h.frames()) == 3


def test_late_compute_is_fenced_then_discarded_before_next_reset(tmp_path):
    h = Harness(tmp_path, timeout=.05)
    try:
        h.form()
        h.delay = .2
        assert not h.call(scene(2, 5.21)).updated_agents.agents
        assert h.proxy._pending is not None and h.states() == ("SOCIAL", "SOCIAL")
        count = len(h.calls)
        assert not h.call(scene(2.1, 5.21)).updated_agents.agents
        assert len(h.calls) == count
        # Wait for the actual remote operation, never restart on timeout alone.
        deadline = time.monotonic() + 2
        while not h.proxy._pending[0].done() and time.monotonic() < deadline:
            time.sleep(.01)
        assert h.proxy._pending[0].done()
        h.delay = 0
        assert len(h.call(scene(2.2, 5.21)).updated_agents.agents) == 2
        assert h.proxy.reset_attempts == 2 and h.proxy.reset_count == 1
        assert h.frames()[-1]["reset_reason"] == "resynchronize"
    finally:
        h.close()


def test_clock_rollback_resets_pair_and_both_profiles(harness):
    h = harness
    h.form()
    h.call(scene(2, 5.21))
    h.call(scene(.5))
    assert h.states() == ("NORMAL", "NORMAL") and not h.proxy.committed.pair.active
    assert h.proxy.committed.epoch == 1 and h.proxy.reset_count == 2
    assert all(item["cause"] == "TIME_RESET" for item in h.frames()[-1]["transitions"])


def test_late_reset_is_not_mistaken_for_remote_cancellation(tmp_path):
    h = Harness(tmp_path, timeout=.05)
    try:
        h.form()
        h.reset_delay = .2
        assert not h.call(scene(2, 5.21)).updated_agents.agents
        assert h.proxy._pending[1] == "reset" and h.states() == ("SOCIAL", "SOCIAL")
        assert [kind for kind, _ in h.calls] == ["compute", "compute", "reset"]
        assert not h.call(scene(2.1, 5.21)).updated_agents.agents
        assert len(h.calls) == 3
        deadline = time.monotonic() + 2
        while not h.proxy._pending[0].done() and time.monotonic() < deadline:
            time.sleep(.01)
        assert h.proxy._pending[0].done()
        h.reset_delay = 0
        assert len(h.call(scene(2.2, 5.21)).updated_agents.agents) == 2
        assert h.proxy.backend_generation == h.proxy.reset_attempts == 2
        assert h.proxy.reset_count == 1 and h.frames()[-1]["reset_reason"] == "resynchronize"
    finally:
        h.close()


def test_shared_flag_off_commits_two_v1_contexts_without_pair_events(tmp_path):
    h = Harness(tmp_path, shared=False)
    try:
        h.call(scene(0))
        h.call(scene(1))
        assert h.states() == ("ATTENTION", "ATTENTION")
        assert h.proxy.reset_count == 0 and not h.proxy.committed.pair.active
        assert not any(frame["shared_events"] for frame in h.frames())
        assert verify_replay(CONFIG.read_bytes(), (h.tmp_path / "steps.jsonl").read_text().splitlines()) == 2
    finally:
        h.close()


def test_telemetry_exception_never_retries_successful_compute(harness):
    h = harness
    h.form()
    def broken(_):
        raise OSError("injected log failure")
    h.proxy._publish = broken
    assert len(h.call(scene(2, 5.21)).updated_agents.agents) == 2
    assert h.states() == ("SURPRISED", "SURPRISED")
    assert not h.proxy.backend_dirty and h.proxy.reset_count == 1
    for _ in range(30):
        assert len(h.call(scene(2, 5.21)).updated_agents.agents) == 2
    assert len(h.calls) == 4


def test_disabled_is_raw_passthrough_without_formal_side_effects(tmp_path):
    h = Harness(tmp_path, enabled=False)
    try:
        result = h.call(scene(0, 5.21))
        assert [agent.id for agent in result.updated_agents.agents] == [2, 1]
        assert h.proxy.commit_seq == 0 and h.proxy.reset_attempts == 0
        assert [kind for kind, _ in h.calls] == ["compute"]
        assert not (tmp_path / "steps.jsonl").exists()
    finally:
        h.close()
