"""Transactional two-human HuNav proxy with late-call quarantine."""

from __future__ import annotations

import copy
import hashlib
import math
from pathlib import Path
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import String
from hunav_msgs.srv import ComputeAgents, ResetAgents
import yaml

from ..behavior_adapter import apply_profile_to_agent, regular_goal_reached, stop_agent_motion
from ..proxy_node import _agent_profile_signature, _kinematics, _service_timeout_seconds, _stamp_to_ns
from ..proxy_validation import _validate_agent, validate_compute_response
from ..trace_serialization import json_text
from .automaton import MultiAgentAutomaton
from .config import MultiConfig
from .model import SceneSnapshot
from .model_export import model_hash
from .trace import semantic_payload


PREFIX = "/formal_social_behavior/multi"
STATIC_FIELDS = ("skin", "group_id", "desired_velocity", "radius", "cyclic_goals", "goal_radius")


def request_fingerprint(request):
    """Hash canonical field values, never transport bytes/alignment padding.

    The caller has already sorted human IDs. Include every request field so
    changed goals/static/profile fields still conflict at a duplicate stamp.
    """
    return hashlib.sha256(json_text(message_to_ordereddict(request)).encode("utf-8")).hexdigest()


class MultiAgentProxy(Node):
    def __init__(self, *, node_name="formal_social_multi_proxy", parameter_overrides=None):
        super().__init__(node_name, parameter_overrides=parameter_overrides)
        defaults = {"enabled": False, "shared_events_enabled": False, "config_file": "",
                    "trace_file": "", "steps_file": "", "backend_trace_file": "",
                    "compute_service": "/compute_agents", "raw_compute_service": PREFIX + "/compute_agents_raw",
                    "raw_reset_service": PREFIX + "/reset_agents_raw", "topic_prefix": PREFIX,
                    "service_timeout_seconds": 10.0}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.enabled = self.get_parameter("enabled").value
        self.shared_enabled = self.get_parameter("shared_events_enabled").value
        self.timeout = _service_timeout_seconds(self.get_parameter("service_timeout_seconds").value)
        config_path = self.get_parameter("config_file").value
        if config_path:
            content = Path(config_path).read_bytes()
            self.config = MultiConfig.from_mapping(yaml.safe_load(content))
            self.config_sha256 = hashlib.sha256(content).hexdigest()
        elif self.enabled:
            raise RuntimeError("config_file is required when enabled=true")
        else:
            self.config, self.config_sha256 = MultiConfig(), "defaults"
        self.model_sha256 = model_hash(self.config)
        self.engine = MultiAgentAutomaton(self.config, shared_events_enabled=self.shared_enabled)
        self.committed = self.engine.initial_context()
        self.profiles = {}
        self.identities = {}
        self.reset_count = 0
        self.reset_attempts = 0
        self.backend_generation = 0
        self.commit_seq = 0
        self.backend_dirty = False
        self._pending = None
        self._cached_hash = None
        self._cached_response = None
        self._shutdown = threading.Event()
        self._lock = threading.Lock()
        self._raw_client_group = ReentrantCallbackGroup()
        self._server_group = MutuallyExclusiveCallbackGroup()
        self.raw_compute = self.create_client(ComputeAgents, self.get_parameter("raw_compute_service").value,
                                              callback_group=self._raw_client_group)
        self.raw_reset = self.create_client(ResetAgents, self.get_parameter("raw_reset_service").value,
                                            callback_group=self._raw_client_group)
        self.server = self.create_service(ComputeAgents, self.get_parameter("compute_service").value,
                                           self._on_compute, callback_group=self._server_group)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        prefix = self.get_parameter("topic_prefix").value.rstrip("/")
        self.publishers_by_kind = {kind: self.create_publisher(String, prefix + "/" + kind, qos)
                                  for kind in ("states", "events", "shared_events", "transitions")}
        self.streams = {}
        for parameter in ("trace_file", "steps_file", "backend_trace_file"):
            path = self.get_parameter(parameter).value
            if path and self.enabled:
                file = Path(path)
                file.parent.mkdir(parents=True, exist_ok=True)
                self.streams[parameter] = file.open("a", encoding="utf-8", buffering=1)
        self.get_logger().info(f"FORMAL_SOCIAL_MULTI_PROXY_READY enabled={self.enabled} "
                               f"shared_events_enabled={self.shared_enabled} ids={self.config.agent_ids} "
                               f"config_sha256={self.config_sha256} model_sha256={self.model_sha256}")

    def _write(self, stream, payload):
        if stream in self.streams:
            self.streams[stream].write(json_text(payload) + "\n")
            self.streams[stream].flush()

    def _operation_log(self, kind, outcome):
        try:
            self._write("backend_trace_file", {"operation": kind, "outcome": outcome,
                        "reset_attempts": self.reset_attempts, "backend_generation": self.backend_generation,
                        "commit_seq": self.commit_seq})
        except Exception as exc:
            self.get_logger().error(f"formal multi telemetry failed: {exc}")

    def _consume_pending(self, *, late=False):
        future, kind = self._pending
        self._pending = None
        try:
            result = future.result()
            if kind == "reset" and result is not None and result.ok:
                self.backend_generation += 1
            self._operation_log(kind, "late_discarded" if late else "returned")
            return result
        except Exception:
            self._operation_log(kind, "exception")
            raise

    def _call(self, client, request, kind):
        if self._pending is not None:
            raise RuntimeError("raw operation still pending; backend quarantined")
        if not client.service_is_ready() and not client.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError(f"timeout waiting for raw {kind}")
        if kind == "reset":
            self.reset_attempts += 1
        future = client.call_async(request)
        self._pending = (future, kind)
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        deadline = time.monotonic() + self.timeout
        while not future.done():
            if self._shutdown.is_set():
                raise RuntimeError("proxy shutdown requested")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self._operation_log(kind, "timeout_quarantined")
                # Local cancellation does not cancel the remote C++ operation.
                raise RuntimeError(f"timeout calling raw {kind}; backend quarantined")
            completed.wait(min(.05, remaining))
        return self._consume_pending()

    def _drain_pending(self):
        if self._pending is None:
            return True
        if not self._pending[0].done():
            return False
        try:
            self._consume_pending(late=True)
        except Exception as exc:
            self.get_logger().warning(f"discarded late raw exception: {exc}")
        self.backend_dirty = True
        return True

    @staticmethod
    def _validate_raw(raw, expected):
        if raw is None or not hasattr(raw, "updated_agents"):
            raise RuntimeError("raw compute returned no response")
        ids = [agent.id for agent in raw.updated_agents.agents]
        if len(set(ids)) != len(ids) or set(ids) != {agent.id for agent in expected}:
            raise RuntimeError("raw compute changed/duplicated agent IDs")
        copied = copy.deepcopy(raw)
        by_id = {agent.id: agent for agent in copied.updated_agents.agents}
        copied.updated_agents.agents = [by_id[agent.id] for agent in expected]
        validate_compute_response(copied, expected_agents=expected)
        for agent in copied.updated_agents.agents:
            for field in ("duration", "vel", "dist", "goal_force_factor", "obstacle_force_factor",
                          "social_force_factor", "other_force_factor"):
                if not math.isfinite(getattr(agent.behavior, field)):
                    raise RuntimeError("non-finite raw behavior profile")
        return copied

    @staticmethod
    def _stop_regular(agents):
        for index, agent in enumerate(agents):
            if agent.behavior.type == 1 and regular_goal_reached(agent):
                agents[index] = stop_agent_motion(agent)

    def _validate_input(self, request):
        if request.current_agents.header.frame_id != "map":
            raise ValueError("Phase 4 requires map frame")
        if request.robot.id != 0 or request.robot.type != request.robot.ROBOT:
            raise ValueError("Phase 4 requires robot ID 0")
        agents = request.current_agents.agents
        ids = [agent.id for agent in agents]
        if len(ids) != 2 or tuple(sorted(ids)) != self.config.agent_ids:
            raise ValueError("exactly the configured two human IDs are required")
        if len({agent.name for agent in agents}) != 2 or any(not agent.name for agent in agents):
            raise ValueError("human names must be unique and nonempty")
        for agent in [request.robot, *agents]:
            _validate_agent(agent)
            if agent.radius < 0 or agent.goal_radius < 0 or agent.desired_velocity < 0:
                raise ValueError("radius, goal radius and desired velocity must be nonnegative")
        for agent in agents:
            if agent.type != agent.PERSON:
                raise ValueError("configured agents must be humans")
            if agent.id in self.identities and agent.name != self.identities[agent.id]:
                raise ValueError("human identity changed")

    def _on_compute(self, request, response):
        with self._lock:
            touched_backend = False
            try:
                if not self._drain_pending():
                    return response
                if not self.enabled:
                    raw = self._call(self.raw_compute, copy.deepcopy(request), "compute")
                    self._validate_raw(raw, request.current_agents.agents)
                    response.updated_agents = copy.deepcopy(raw.updated_agents)
                    return response
                self._validate_input(request)
                order = [agent.id for agent in request.current_agents.agents]
                candidate = copy.deepcopy(request)
                candidate.current_agents.agents = sorted(candidate.current_agents.agents, key=lambda agent: agent.id)
                fingerprint = request_fingerprint(candidate)
                snapshot = SceneSnapshot(_stamp_to_ns(candidate.current_agents.header.stamp),
                    _kinematics(candidate.robot), tuple((agent.id, _kinematics(agent)) for agent in candidate.current_agents.agents))
                if self.committed.last_snapshot and snapshot.sim_time_ns == self.committed.last_snapshot.sim_time_ns:
                    if fingerprint != self._cached_hash:
                        raise ValueError("conflicting request at the same simulation timestamp")
                    cached = copy.deepcopy(self._cached_response)
                    by_id = {agent.id: agent for agent in cached.agents}
                    cached.agents = [by_id[agent_id] for agent_id in order]
                    response.updated_agents = cached
                    return response
                step = self.engine.evaluate(self.committed, snapshot)
                signatures, changed = {}, []
                names = {agent.id: agent.name for agent in candidate.current_agents.agents}
                for index, (agent, memory) in enumerate(zip(candidate.current_agents.agents, step.context.agents)):
                    adapted = apply_profile_to_agent(agent, self.config.behavior_profiles[memory.context.state])
                    signature = _agent_profile_signature(adapted)
                    if signature != self.profiles.get(agent.id, _agent_profile_signature(agent)):
                        changed.append(agent.id)
                    signatures[agent.id] = signature
                    candidate.current_agents.agents[index] = adapted
                self._stop_regular(candidate.current_agents.agents)
                reset_needed = bool(changed) or self.backend_dirty
                reason = "resynchronize" if self.backend_dirty else "profile_change"
                touched_backend = True
                if reset_needed:
                    reset = ResetAgents.Request()
                    reset.current_agents = copy.deepcopy(candidate.current_agents)
                    reset.robot = copy.deepcopy(candidate.robot)
                    result = self._call(self.raw_reset, reset, "reset")
                    if result is None or not result.ok:
                        raise RuntimeError("raw reset rejected candidate batch")
                raw = self._call(self.raw_compute, candidate, "compute")
                ordered = self._validate_raw(raw, candidate.current_agents.agents)
                updated = ordered.updated_agents
                for output, source in zip(updated.agents, candidate.current_agents.agents):
                    for field in STATIC_FIELDS:
                        setattr(output, field, getattr(source, field))
                self._stop_regular(updated.agents)
                frame = semantic_payload(snapshot, step, config_sha256=self.config_sha256,
                                         model_sha256=self.model_sha256, names=names)
                frame.update(commit_seq=self.commit_seq + 1, reset_count=self.reset_count + int(reset_needed),
                    backend_generation=self.backend_generation, changed_profile_ids=changed,
                    reset_affected_ids=list(self.config.agent_ids) if reset_needed else [],
                    reset_reason=reason if reset_needed else None,
                    shared_events_enabled=self.shared_enabled)
                for payload, memory in zip(frame["agents"], step.context.agents):
                    payload["profile"] = self.config.behavior_profiles[memory.context.state].as_dict()
                json_text(frame)  # Materialize/validate before committing; I/O follows commit.
                cached_response = copy.deepcopy(updated)
                by_id = {agent.id: agent for agent in updated.agents}
                updated.agents = [by_id[agent_id] for agent_id in order]
                self.committed = step.context
                self.identities = names
                self.profiles = signatures
                self.commit_seq += 1
                self.reset_count += int(reset_needed)
                self.backend_dirty = False
                self._cached_hash, self._cached_response = fingerprint, cached_response
                response.updated_agents = updated
                try:
                    self._publish(frame)
                except Exception as exc:
                    self.get_logger().error(f"formal multi telemetry failed: {exc}")
                return response
            except Exception as exc:
                if touched_backend:
                    self.backend_dirty = True
                if not self._shutdown.is_set() and self.context.ok():
                    self.get_logger().error(f"formal multi compute failed: {exc}")
                return response

    def _publish(self, frame):
        self._write("steps_file", frame)
        for kind in ("states", "events", "shared_events"):
            self.publishers_by_kind[kind].publish(String(data=json_text({**frame, "message_type": kind})))
        if frame["transitions"]:
            transition = {**frame, "message_type": "transitions"}
            self.publishers_by_kind["transitions"].publish(String(data=json_text(transition)))
            self._write("trace_file", transition)
            self.get_logger().info("FORMAL_SOCIAL_MULTI_TRANSITION " + json_text({
                "sim_time_ns": frame["sim_time_ns"], "transitions": frame["transitions"],
                "reset_count": frame["reset_count"]}))

    def request_shutdown(self):
        self._shutdown.set()

    def destroy_node(self):
        self.request_shutdown()
        for stream in self.streams.values():
            stream.close()
        self.streams.clear()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiAgentProxy()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.request_shutdown()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
