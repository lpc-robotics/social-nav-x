"""Immutable two-human values. No ROS or wall-clock dependencies."""

from __future__ import annotations

from dataclasses import dataclass, fields
from enum import Enum

from ..model import AutomatonContext, EventMemory, EventSnapshot, FormalState, PlanarKinematics


class MultiState(str, Enum):
    NORMAL = "NORMAL"
    ATTENTION = "ATTENTION"
    CURIOUS = "CURIOUS"
    SURPRISED = "SURPRISED"
    SCARED = "SCARED"
    SOCIAL = "SOCIAL"


class PeerEvent(str, Enum):
    PEER_VISIBLE = "PEER_VISIBLE"
    PEER_NEAR = "PEER_NEAR"
    MUTUAL_GAZE = "MUTUAL_GAZE"
    SOCIAL_SPACE_FORMED = "SOCIAL_SPACE_FORMED"
    SOCIAL_SPACE_BROKEN = "SOCIAL_SPACE_BROKEN"
    ROBOT_INTRUSION = "ROBOT_INTRUSION"


@dataclass(frozen=True)
class MultiAgentAutomatonContext:
    state: MultiState = MultiState.NORMAL
    state_entered_ns: int | None = None
    safe_since_ns: int | None = None
    cooldown_until_ns: int | None = None
    last_stamp_ns: int | None = None
    last_transition_stamp_ns: int | None = None

    def __post_init__(self):
        object.__setattr__(self, "state", MultiState(self.state))
        for field in fields(self):
            value = getattr(self, field.name)
            if field.name != "state" and value is not None:
                if type(value) is not int or value < 0:
                    raise ValueError(f"{field.name} must be nonnegative integer nanoseconds")

    def as_v1(self) -> AutomatonContext:
        return AutomatonContext(**{
            field.name: FormalState(self.state.value) if field.name == "state"
            else getattr(self, field.name) for field in fields(self)
        })

    @classmethod
    def from_v1(cls, context: AutomatonContext):
        return cls(**{
            field.name: MultiState(context.state.value) if field.name == "state"
            else getattr(context, field.name) for field in fields(cls)
        })


@dataclass(frozen=True)
class SceneSnapshot:
    sim_time_ns: int
    robot: PlanarKinematics
    humans: tuple[tuple[int, PlanarKinematics], ...]

    def __post_init__(self):
        if type(self.sim_time_ns) is not int or self.sim_time_ns < 0:
            raise ValueError("sim_time_ns must be nonnegative integer nanoseconds")
        ids = [agent_id for agent_id, _ in self.humans]
        if len(ids) != 2 or any(type(i) is not int or i <= 0 for i in ids):
            raise ValueError("exactly two positive integer human IDs are required")
        if len(set(ids)) != 2:
            raise ValueError("duplicate human ID")
        if not isinstance(self.robot, PlanarKinematics) or any(
            not isinstance(human, PlanarKinematics) for _, human in self.humans
        ):
            raise TypeError("scene motion must be PlanarKinematics")
        object.__setattr__(self, "humans", tuple(sorted(self.humans)))


@dataclass(frozen=True)
class AgentMemory:
    agent_id: int
    context: MultiAgentAutomatonContext = MultiAgentAutomatonContext()
    robot_events: EventMemory = EventMemory()


@dataclass(frozen=True)
class PairMemory:
    visible: tuple[bool, bool] = (False, False)
    near: bool = False
    gaze: bool = False
    stationary: bool = False
    active: bool = False
    session: int = 0
    ready_since_ns: int | None = None
    broken_since_ns: int | None = None
    cooldown_until_ns: int | None = None
    anchors: tuple[tuple[float, float], tuple[float, float]] | None = None
    intrusion_active: bool = False
    previous_robot: tuple[float, float] | None = None


@dataclass(frozen=True)
class PairGeometry:
    distance: float
    bearing_errors: tuple[float, float]
    visible: tuple[bool, bool]
    near: bool
    gaze: bool
    stationary: bool
    valid: bool
    space_distance: float
    swept_distance: float
    intrusion_active: bool


@dataclass(frozen=True)
class SharedEvent:
    name: PeerEvent
    event_id: str
    participants: tuple[int, int]
    reason: str
    triggered_by_ids: tuple[int, ...] = ()


@dataclass(frozen=True)
class PairEvaluation:
    memory: PairMemory
    geometry: PairGeometry
    shared_events: tuple[SharedEvent, ...] = ()


@dataclass(frozen=True)
class MultiContext:
    agents: tuple[AgentMemory, ...]
    pair: PairMemory = PairMemory()
    epoch: int = 0
    last_snapshot: SceneSnapshot | None = None


@dataclass(frozen=True)
class MultiTransition:
    agent_id: int
    old_state: MultiState
    new_state: MultiState
    cause: str
    rule_id: str
    shared_event_ids: tuple[str, ...] = ()


@dataclass(frozen=True)
class MultiStep:
    context: MultiContext
    robot_events: tuple[tuple[int, EventSnapshot], ...]
    pair_evaluation: PairEvaluation | None
    transitions: tuple[MultiTransition, ...] = ()
    duplicate: bool = False
    clock_reset: bool = False
