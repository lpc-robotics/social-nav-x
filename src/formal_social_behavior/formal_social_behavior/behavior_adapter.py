"""HuNav behavior profiles without importing HuNav or ROS message packages."""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass
from enum import IntEnum
from types import MappingProxyType
from typing import Any, Mapping, MutableMapping, Tuple

from .model import FormalState


class BehaviorType(IntEnum):
    """HuNav v1 behavior type constants used by the V1 automaton."""

    REGULAR = 1
    SURPRISED = 3
    SCARED = 4
    CURIOUS = 5


@dataclass(frozen=True)
class BehaviorProfile:
    """Complete configurable ``hunav_msgs/AgentBehavior`` profile."""

    behavior_type: BehaviorType
    state: int = 0
    configuration: int = 1
    duration: float = 40.0
    once: bool = True
    vel: float = 0.6
    dist: float = 0.0
    goal_force_factor: float = 2.0
    obstacle_force_factor: float = 10.0
    social_force_factor: float = 5.0
    other_force_factor: float = 20.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "behavior_type", BehaviorType(self.behavior_type))
        if not isinstance(self.state, int) or isinstance(self.state, bool):
            raise TypeError("state must be an integer")
        if not isinstance(self.configuration, int) or isinstance(
            self.configuration, bool
        ):
            raise TypeError("configuration must be an integer")
        if not isinstance(self.once, bool):
            raise TypeError("once must be a boolean")
        for name in (
            "duration",
            "vel",
            "dist",
            "goal_force_factor",
            "obstacle_force_factor",
            "social_force_factor",
            "other_force_factor",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
            if value < 0.0:
                raise ValueError(f"{name} must be non-negative")

    @property
    def type(self) -> int:
        """Integer alias matching the HuNav message field name."""

        return int(self.behavior_type)

    def signature(self) -> Tuple[Any, ...]:
        """Stable tuple for deciding whether HuNav needs one reset."""

        return (
            int(self.behavior_type),
            self.state,
            self.configuration,
            self.duration,
            self.once,
            self.vel,
            self.dist,
            self.goal_force_factor,
            self.obstacle_force_factor,
            self.social_force_factor,
            self.other_force_factor,
        )

    def as_dict(self) -> dict[str, Any]:
        return {
            "type": int(self.behavior_type),
            "state": self.state,
            "configuration": self.configuration,
            "duration": self.duration,
            "once": self.once,
            "vel": self.vel,
            "dist": self.dist,
            "goal_force_factor": self.goal_force_factor,
            "obstacle_force_factor": self.obstacle_force_factor,
            "social_force_factor": self.social_force_factor,
            "other_force_factor": self.other_force_factor,
        }

    @classmethod
    def from_mapping(
        cls,
        values: Mapping[str, Any],
        *,
        base: "BehaviorProfile | None" = None,
    ) -> "BehaviorProfile":
        merged: MutableMapping[str, Any] = {}
        if base is not None:
            merged.update(base.as_dict())
        merged.update(values)
        if "behavior_type" in merged:
            merged["type"] = merged.pop("behavior_type")
        try:
            behavior_type = BehaviorType(int(merged.pop("type")))
        except KeyError as exc:
            raise ValueError("behavior profile requires 'type'") from exc
        return cls(behavior_type=behavior_type, **merged)


def default_behavior_profiles() -> Mapping[FormalState, BehaviorProfile]:
    """Return the deterministic profiles verified by the six-behavior demo."""

    regular = BehaviorProfile(behavior_type=BehaviorType.REGULAR)
    profiles = {
        FormalState.NORMAL: regular,
        FormalState.ATTENTION: regular,
        FormalState.CURIOUS: BehaviorProfile(
            behavior_type=BehaviorType.CURIOUS,
            duration=30.0,
            once=False,
            vel=0.8,
            dist=1.5,
        ),
        FormalState.SURPRISED: BehaviorProfile(
            behavior_type=BehaviorType.SURPRISED,
            duration=30.0,
            once=False,
            vel=0.6,
            dist=4.0,
        ),
        FormalState.SCARED: BehaviorProfile(
            behavior_type=BehaviorType.SCARED,
            duration=40.0,
            once=False,
            vel=0.6,
            dist=3.0,
        ),
    }
    return MappingProxyType(profiles)


def profile_for_state(
    state: FormalState,
    profiles: Mapping[FormalState, BehaviorProfile] | None = None,
) -> BehaviorProfile:
    selected = default_behavior_profiles() if profiles is None else profiles
    normalized_state = FormalState(state)
    try:
        return selected[normalized_state]
    except KeyError as exc:
        raise ValueError(f"no HuNav profile configured for {normalized_state.value}") from exc


def profile_signature(profile: BehaviorProfile) -> Tuple[Any, ...]:
    return profile.signature()


def apply_profile_to_agent(agent: Any, profile: BehaviorProfile) -> Any:
    """Deep-copy ``agent`` and apply every behavior-profile field to the copy."""

    candidate = copy.deepcopy(agent)
    if not hasattr(candidate, "behavior"):
        raise TypeError("agent must expose a 'behavior' attribute")
    behavior = candidate.behavior
    for name, value in profile.as_dict().items():
        if not hasattr(behavior, name):
            raise TypeError(f"agent.behavior does not expose {name!r}")
        setattr(behavior, name, value)
    return candidate
