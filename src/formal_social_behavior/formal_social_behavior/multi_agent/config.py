"""Strict Phase 4 configuration, separate from the unchanged V1 parser."""

from dataclasses import dataclass, fields
from types import MappingProxyType
from typing import Mapping

from ..behavior_adapter import BehaviorProfile, default_behavior_profiles
from ..config import AutomatonTiming, EventThresholds, _finite_number
from .model import MultiState


@dataclass(frozen=True)
class PeerThresholds:
    visible_enter_distance: float = 6.0
    visible_exit_distance: float = 6.5
    visible_enter_half_fov_deg: float = 100.0
    visible_exit_half_fov_deg: float = 110.0
    near_enter_distance: float = 2.5
    near_exit_distance: float = 2.8
    gaze_enter_deg: float = 25.0
    gaze_exit_deg: float = 35.0
    stationary_enter_speed: float = 0.05
    stationary_exit_speed: float = 0.10
    intrusion_enter_margin: float = 0.45
    intrusion_exit_margin: float = 0.65
    formation_dwell_seconds: float = 1.0
    break_dwell_seconds: float = 0.3
    reentry_cooldown_seconds: float = 1.0

    def __post_init__(self):
        for field in fields(self):
            value = _finite_number(field.name, getattr(self, field.name))
            if value < 0:
                raise ValueError(f"{field.name} must be nonnegative")
            object.__setattr__(self, field.name, value)
        for stem in ("visible", "near"):
            EventThresholds._validate_band(stem, getattr(self, stem + "_enter_distance"),
                                           getattr(self, stem + "_exit_distance"))
        for enter, exit_ in (("visible_enter_half_fov_deg", "visible_exit_half_fov_deg"),
                             ("gaze_enter_deg", "gaze_exit_deg"),
                             ("stationary_enter_speed", "stationary_exit_speed"),
                             ("intrusion_enter_margin", "intrusion_exit_margin")):
            EventThresholds._validate_band(enter, getattr(self, enter), getattr(self, exit_))
        if not 0 < self.visible_exit_half_fov_deg <= 180 or self.gaze_exit_deg > 180:
            raise ValueError("FOV and gaze bounds must be <=180 degrees")
        if self.visible_enter_half_fov_deg <= 0:
            raise ValueError("visible FOV must be positive")

    @classmethod
    def from_mapping(cls, values):
        if not isinstance(values, Mapping):
            raise TypeError("peer must be a mapping")
        unknown = set(values) - {field.name for field in fields(cls)}
        if unknown:
            raise ValueError(f"unknown peer keys: {sorted(unknown, key=str)}")
        return cls(**values)


def default_profiles():
    profiles = {MultiState(state.value): value for state, value in default_behavior_profiles().items()}
    profiles[MultiState.SOCIAL] = profiles[MultiState.NORMAL]
    return profiles


@dataclass(frozen=True)
class MultiConfig:
    agent_ids: tuple[int, int] = (1, 2)
    thresholds: EventThresholds = EventThresholds()
    timing: AutomatonTiming = AutomatonTiming()
    peer: PeerThresholds = PeerThresholds()
    behavior_profiles: Mapping | None = None

    def __post_init__(self):
        ids = self.agent_ids
        if not isinstance(ids, (tuple, list)) or len(ids) != 2 or any(
            type(value) is not int or value <= 0 for value in ids
        ) or len(set(ids)) != 2:
            raise ValueError("agent_ids must contain exactly two distinct positive integers")
        object.__setattr__(self, "agent_ids", tuple(sorted(ids)))
        for key, expected in (("thresholds", EventThresholds), ("timing", AutomatonTiming),
                              ("peer", PeerThresholds)):
            if not isinstance(getattr(self, key), expected):
                raise TypeError(f"{key} must be {expected.__name__}")
        profiles = default_profiles() if self.behavior_profiles is None else self.behavior_profiles
        normalized = {MultiState(key): value for key, value in profiles.items()}
        if set(normalized) != set(MultiState) or any(
            not isinstance(value, BehaviorProfile) for value in normalized.values()
        ):
            raise ValueError("behavior_profiles must cover all six states with complete profiles")
        if not normalized[MultiState.SOCIAL] == normalized[MultiState.NORMAL] == normalized[MultiState.ATTENTION]:
            raise ValueError("SOCIAL, NORMAL and ATTENTION must share the same Regular profile")
        if normalized[MultiState.SOCIAL].type != 1:
            raise ValueError("SOCIAL requires Regular type 1")
        object.__setattr__(self, "behavior_profiles", MappingProxyType(normalized))

    @classmethod
    def from_mapping(cls, document):
        if not isinstance(document, Mapping):
            raise TypeError("Phase 4 config must be a mapping")
        root = document
        for wrapper in ("formal_social_multi", "ros__parameters"):
            if wrapper in root:
                if set(root) != {wrapper} or not isinstance(root[wrapper], Mapping):
                    raise ValueError(f"invalid {wrapper} wrapper")
                root = root[wrapper]
        unknown = set(root) - {field.name for field in fields(cls)}
        if unknown:
            raise ValueError(f"unknown Phase 4 config keys: {sorted(unknown, key=str)}")
        profiles = None
        if "behavior_profiles" in root:
            raw = root["behavior_profiles"]
            if not isinstance(raw, Mapping):
                raise TypeError("behavior_profiles must be a mapping")
            profiles = {}
            defaults = default_profiles()
            for key, value in raw.items():
                state = MultiState(str(key).upper())
                if state in profiles:
                    raise ValueError(f"duplicate normalized profile {state.value}")
                if not isinstance(value, Mapping):
                    raise TypeError("profile must be a mapping")
                profiles[state] = BehaviorProfile.from_mapping(value, base=defaults[state])
        for key in ("thresholds", "timing"):
            if not isinstance(root.get(key, {}), Mapping):
                raise TypeError(f"{key} must be a mapping")
        return cls(agent_ids=root.get("agent_ids", (1, 2)),
                   thresholds=EventThresholds.from_mapping(root.get("thresholds", {})),
                   timing=AutomatonTiming.from_mapping(root.get("timing", {})),
                   peer=PeerThresholds.from_mapping(root.get("peer", {})),
                   behavior_profiles=profiles)
