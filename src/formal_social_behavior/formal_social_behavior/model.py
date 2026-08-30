"""ROS-independent data model for the formal social automaton.

The objects in this module are deliberately immutable.  A ROS service proxy can
therefore evaluate one simulation tick, call HuNav, and only retain the returned
candidate objects after the service call succeeds.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import FrozenSet, Optional


class FormalState(str, Enum):
    """V1 social states for one human interacting with one robot."""

    NORMAL = "NORMAL"
    ATTENTION = "ATTENTION"
    CURIOUS = "CURIOUS"
    SURPRISED = "SURPRISED"
    SCARED = "SCARED"


class SocialEvent(str, Enum):
    """Discrete, observable events produced by :class:`EventExtractor`."""

    ROBOT_VISIBLE = "ROBOT_VISIBLE"
    ROBOT_LOST = "ROBOT_LOST"
    ROBOT_NEAR = "ROBOT_NEAR"
    ROBOT_SAFE_APPROACH = "ROBOT_SAFE_APPROACH"
    SUDDEN_NEAR = "SUDDEN_NEAR"
    PERSONAL_SPACE_VIOLATION = "PERSONAL_SPACE_VIOLATION"
    ROBOT_FAST_APPROACH = "ROBOT_FAST_APPROACH"
    TTC_LOW = "TTC_LOW"
    ROBOT_LEAVING = "ROBOT_LEAVING"
    TIME_RESET = "TIME_RESET"


class TransitionCause(str, Enum):
    """Stable cause labels used in transition traces."""

    PERSONAL_SPACE_VIOLATION = SocialEvent.PERSONAL_SPACE_VIOLATION.value
    TTC_LOW = SocialEvent.TTC_LOW.value
    ROBOT_FAST_APPROACH = SocialEvent.ROBOT_FAST_APPROACH.value
    SUDDEN_NEAR = SocialEvent.SUDDEN_NEAR.value
    ROBOT_VISIBLE = SocialEvent.ROBOT_VISIBLE.value
    ROBOT_LOST = SocialEvent.ROBOT_LOST.value
    ROBOT_LEAVING = SocialEvent.ROBOT_LEAVING.value
    ATTENTION_DWELL = "ATTENTION_DWELL"
    RECOVERY_TIMEOUT = "RECOVERY_TIMEOUT"
    TIME_RESET = SocialEvent.TIME_RESET.value


def _require_finite(name: str, value: float) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value!r}")


def _require_stamp(name: str, value: Optional[int]) -> None:
    if value is not None and (not isinstance(value, int) or isinstance(value, bool)):
        raise TypeError(f"{name} must be an integer nanosecond stamp or None")
    if value is not None and value < 0:
        raise ValueError(f"{name} must be non-negative")


@dataclass(frozen=True)
class PlanarKinematics:
    """Position, velocity, heading, and collision radius in the map plane."""

    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    yaw: float = 0.0
    radius: float = 0.0

    def __post_init__(self) -> None:
        for name in ("x", "y", "vx", "vy", "yaw", "radius"):
            _require_finite(name, float(getattr(self, name)))
        if self.radius < 0.0:
            raise ValueError("radius must be non-negative")


@dataclass(frozen=True)
class MotionSnapshot:
    """One time-stamped robot/human observation in the map frame."""

    sim_time_ns: int
    robot: PlanarKinematics
    human: PlanarKinematics

    def __post_init__(self) -> None:
        _require_stamp("sim_time_ns", self.sim_time_ns)


@dataclass(frozen=True)
class InteractionMetrics:
    """Continuous interaction quantities from one observation."""

    distance: float
    closing_speed: float
    ttc_seconds: float
    bearing_to_robot: float
    relative_bearing: float

    def __post_init__(self) -> None:
        for name in (
            "distance",
            "closing_speed",
            "bearing_to_robot",
            "relative_bearing",
        ):
            _require_finite(name, float(getattr(self, name)))
        if self.distance < 0.0:
            raise ValueError("distance must be non-negative")
        if math.isnan(self.ttc_seconds) or self.ttc_seconds < 0.0:
            raise ValueError("ttc_seconds must be non-negative or infinity")


@dataclass(frozen=True)
class EventMemory:
    """Schmitt-trigger latch state carried between committed observations."""

    robot_visible: bool = False
    robot_near: bool = False
    personal_space_violation: bool = False
    robot_fast_approach: bool = False
    ttc_low: bool = False
    robot_leaving: bool = False
    last_stamp_ns: Optional[int] = None

    def __post_init__(self) -> None:
        _require_stamp("last_stamp_ns", self.last_stamp_ns)


@dataclass(frozen=True)
class EventSnapshot:
    """Events and metrics calculated for a single simulation timestamp."""

    sim_time_ns: int
    events: FrozenSet[SocialEvent]
    metrics: InteractionMetrics
    clock_rollback: bool = False

    def __post_init__(self) -> None:
        _require_stamp("sim_time_ns", self.sim_time_ns)
        normalized = frozenset(SocialEvent(event) for event in self.events)
        object.__setattr__(self, "events", normalized)

    def has(self, event: SocialEvent) -> bool:
        return event in self.events


@dataclass(frozen=True)
class EventEvaluation:
    """A transactional event-extraction result."""

    event_snapshot: EventSnapshot
    memory: EventMemory


@dataclass(frozen=True)
class Transition:
    """One committed candidate transition (at most one per timestamp)."""

    sim_time_ns: int
    old_state: FormalState
    new_state: FormalState
    cause: TransitionCause

    def __post_init__(self) -> None:
        _require_stamp("sim_time_ns", self.sim_time_ns)


@dataclass(frozen=True)
class AutomatonContext:
    """Timers and state needed to evaluate the next automaton tick."""

    state: FormalState = FormalState.NORMAL
    state_entered_ns: Optional[int] = None
    safe_since_ns: Optional[int] = None
    cooldown_until_ns: Optional[int] = None
    last_stamp_ns: Optional[int] = None
    last_transition_stamp_ns: Optional[int] = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "state", FormalState(self.state))
        for name in (
            "state_entered_ns",
            "safe_since_ns",
            "cooldown_until_ns",
            "last_stamp_ns",
            "last_transition_stamp_ns",
        ):
            _require_stamp(name, getattr(self, name))


@dataclass(frozen=True)
class AutomatonStep:
    """A transactional automaton result returned without mutating its input."""

    context: AutomatonContext
    event_snapshot: EventSnapshot
    transition: Optional[Transition] = None
    clock_reset: bool = False
