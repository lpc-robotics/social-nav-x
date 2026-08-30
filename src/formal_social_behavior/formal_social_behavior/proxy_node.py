"""Transactional ROS 2 proxy between the Arena bridge and HuNav v1."""

from __future__ import annotations

import copy
import hashlib
from pathlib import Path
import threading
from typing import Any

import rclpy
import yaml
from hunav_msgs.srv import ComputeAgents, ResetAgents
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .automaton import SocialAutomaton
from .behavior_adapter import (
    apply_profile_to_agent,
    profile_for_state,
    profile_signature,
)
from .config import FormalSocialConfig
from .event_extractor import EventExtractor
from .model import (
    EventMemory,
    MotionSnapshot,
    PlanarKinematics,
)
from .proxy_validation import validate_compute_response
from .trace_serialization import (
    json_text,
    observation_payload,
    transition_payload,
)


def _stamp_to_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _kinematics(agent: Any) -> PlanarKinematics:
    return PlanarKinematics(
        x=float(agent.position.position.x),
        y=float(agent.position.position.y),
        vx=float(agent.velocity.linear.x),
        vy=float(agent.velocity.linear.y),
        yaw=float(agent.yaw),
        radius=float(agent.radius),
    )


class FormalSocialBehaviorProxy(Node):
    """Expose ``/compute_agents`` while transactionally driving HuNav modes."""

    def __init__(
        self,
        *,
        node_name: str = "formal_social_behavior_proxy",
        parameter_overrides=None,
    ) -> None:
        super().__init__(node_name, parameter_overrides=parameter_overrides)
        self.declare_parameter("enabled", False)
        self.declare_parameter("config_file", "")
        self.declare_parameter("trace_file", "")
        self.declare_parameter("compute_service", "/compute_agents")
        self.declare_parameter(
            "raw_compute_service",
            "/formal_social_behavior/compute_agents_raw",
        )
        self.declare_parameter(
            "raw_reset_service",
            "/formal_social_behavior/reset_agents_raw",
        )
        self.declare_parameter("service_timeout_seconds", 10.0)

        self._enabled = bool(self.get_parameter("enabled").value)
        self._timeout = float(
            self.get_parameter("service_timeout_seconds").value
        )
        if self._timeout <= 0.0:
            raise RuntimeError("service_timeout_seconds must be positive")

        self._config, self._config_sha256 = self._load_config()
        self._extractor = EventExtractor(self._config.thresholds)
        self._automaton = SocialAutomaton(self._config.timing)
        self._event_memory = EventMemory()
        self._automaton_context = self._automaton.initial_context()
        self._active_profile_signature = None
        self._reset_count = 0

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._states_pub = self.create_publisher(
            String, "/formal_social_behavior/states", qos
        )
        self._events_pub = self.create_publisher(
            String, "/formal_social_behavior/events", qos
        )
        self._transitions_pub = self.create_publisher(
            String, "/formal_social_behavior/transitions", qos
        )

        self._trace_stream = self._open_trace()
        self._client_group = ReentrantCallbackGroup()
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._raw_compute = self.create_client(
            ComputeAgents,
            str(self.get_parameter("raw_compute_service").value),
            callback_group=self._client_group,
        )
        self._raw_reset = self.create_client(
            ResetAgents,
            str(self.get_parameter("raw_reset_service").value),
            callback_group=self._client_group,
        )
        self._compute_server = self.create_service(
            ComputeAgents,
            str(self.get_parameter("compute_service").value),
            self._on_compute,
            callback_group=self._service_group,
        )
        self.get_logger().info(
            "FORMAL_SOCIAL_PROXY_READY "
            f"enabled={str(self._enabled).lower()} "
            f"target_agent_id={self._config.target_agent_id} "
            f"config_sha256={self._config_sha256}"
        )

    def _load_config(self) -> tuple[FormalSocialConfig, str]:
        config_path = str(self.get_parameter("config_file").value)
        if not config_path:
            if self._enabled:
                raise RuntimeError("config_file is required when enabled=true")
            return FormalSocialConfig(), "defaults"
        path = Path(config_path).expanduser().resolve()
        content = path.read_bytes()
        document = yaml.safe_load(content.decode("utf-8"))
        if not isinstance(document, dict):
            raise RuntimeError(f"formal social config must be a mapping: {path}")
        return (
            FormalSocialConfig.from_mapping(document),
            hashlib.sha256(content).hexdigest(),
        )

    def _open_trace(self):
        trace_path = str(self.get_parameter("trace_file").value)
        if not trace_path:
            return None
        path = Path(trace_path).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        return path.open("a", encoding="utf-8", buffering=1)

    def _call(self, client, request, service_name: str):
        if not client.service_is_ready() and not client.wait_for_service(
            timeout_sec=self._timeout
        ):
            raise RuntimeError(f"timeout waiting for {service_name}")
        future = client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not future.done() and not completed.wait(self._timeout):
            future.cancel()
            raise RuntimeError(f"timeout calling {service_name}")
        if future.cancelled():
            raise RuntimeError(f"call cancelled for {service_name}")
        return future.result()

    def _target_index(self, agents) -> int:
        matches = [
            index
            for index, agent in enumerate(agents)
            if int(agent.id) == self._config.target_agent_id
        ]
        if len(matches) != 1:
            raise RuntimeError(
                "expected exactly one target agent "
                f"id={self._config.target_agent_id}, got {len(matches)}"
            )
        return matches[0]

    def _on_compute(self, request, response):
        try:
            if not self._enabled:
                raw = self._call(
                    self._raw_compute,
                    copy.deepcopy(request),
                    "raw compute_agents",
                )
                validate_compute_response(
                    raw,
                    expected_agents=request.current_agents.agents,
                )
                response.updated_agents = copy.deepcopy(raw.updated_agents)
                return response

            candidate_request = copy.deepcopy(request)
            target_index = self._target_index(
                candidate_request.current_agents.agents
            )
            target = candidate_request.current_agents.agents[target_index]
            snapshot = MotionSnapshot(
                sim_time_ns=_stamp_to_ns(
                    candidate_request.current_agents.header.stamp
                ),
                robot=_kinematics(candidate_request.robot),
                human=_kinematics(target),
            )

            event_evaluation = self._extractor.evaluate(
                snapshot, self._event_memory
            )
            automaton_step = self._automaton.step(
                self._automaton_context,
                event_evaluation.event_snapshot,
            )
            profile = profile_for_state(
                automaton_step.context.state,
                self._config.behavior_profiles,
            )
            desired_signature = profile_signature(profile)
            candidate_request.current_agents.agents[target_index] = (
                apply_profile_to_agent(target, profile)
            )

            reset_needed = event_evaluation.event_snapshot.clock_rollback or (
                self._active_profile_signature is not None
                and desired_signature != self._active_profile_signature
            )
            if reset_needed:
                reset_request = ResetAgents.Request()
                reset_request.current_agents = copy.deepcopy(
                    candidate_request.current_agents
                )
                reset_request.robot = copy.deepcopy(candidate_request.robot)
                reset_response = self._call(
                    self._raw_reset,
                    reset_request,
                    "raw reset_agents",
                )
                if reset_response is None or not reset_response.ok:
                    raise RuntimeError(
                        f"raw reset_agents rejected transition: {reset_response}"
                    )

            raw_response = self._call(
                self._raw_compute,
                candidate_request,
                "raw compute_agents",
            )
            validate_compute_response(
                raw_response,
                expected_agents=candidate_request.current_agents.agents,
            )

            # Commit only after every required HuNav operation succeeded.
            self._event_memory = event_evaluation.memory
            self._automaton_context = automaton_step.context
            self._active_profile_signature = desired_signature
            if reset_needed:
                self._reset_count += 1
            response.updated_agents = copy.deepcopy(raw_response.updated_agents)
            try:
                self._publish_observation(
                    target,
                    event_evaluation.event_snapshot,
                    automaton_step,
                    profile.type,
                )
            except Exception as exc:
                # HuNav has already accepted this tick, so telemetry is
                # explicitly best-effort and must not turn success into a
                # bridge retry that would duplicate the compute operation.
                self.get_logger().error(
                    f"formal social telemetry failed: {exc}"
                )
            return response
        except Exception as exc:
            self.get_logger().error(f"formal social compute failed: {exc}")
            # Before the raw compute succeeds, returning an empty response
            # makes the bridge reject this tick and retry; candidate values
            # remain uncommitted. Telemetry failures are handled separately.
            return response

    def _publish_observation(
        self,
        target,
        event_snapshot,
        automaton_step,
        behavior_type: int,
    ) -> None:
        observation = observation_payload(
            event_snapshot=event_snapshot,
            state=automaton_step.context.state,
            behavior_type=behavior_type,
            agent_id=int(target.id),
            agent_name=str(target.name),
            config_sha256=self._config_sha256,
            reset_count=self._reset_count,
        )
        event_payload = {**observation, "message_type": "events"}
        state_payload = {**observation, "message_type": "state"}
        self._events_pub.publish(String(data=json_text(event_payload)))
        self._states_pub.publish(String(data=json_text(state_payload)))

        transition = automaton_step.transition
        if transition is None:
            return
        payload = transition_payload(observation, transition)
        text = json_text(payload)
        self._transitions_pub.publish(String(data=text))
        if self._trace_stream is not None:
            self._trace_stream.write(text + "\n")
            self._trace_stream.flush()
        self.get_logger().info(
            "FORMAL_SOCIAL_TRANSITION "
            f"agent={target.name}/{target.id} "
            f"old={transition.old_state.value} "
            f"new={transition.new_state.value} "
            f"cause={transition.cause.value} resets={self._reset_count}"
        )

    def destroy_node(self):
        if self._trace_stream is not None:
            self._trace_stream.close()
            self._trace_stream = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FormalSocialBehaviorProxy()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
