"""Validated configuration for the ROS-independent automaton core."""

from __future__ import annotations

import math
from dataclasses import dataclass, fields
from types import MappingProxyType
from typing import Any, Mapping

from .behavior_adapter import (
    BehaviorProfile,
    default_behavior_profiles,
)
from .model import FormalState


def _finite_number(name: str, value: Any) -> float:
    """Validate an uncoerced YAML value and return its canonical float."""

    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be an int or float")
    normalized = float(value)
    if not math.isfinite(normalized):
        raise ValueError(f"{name} must be finite")
    return normalized


@dataclass(frozen=True)
class EventThresholds:
    """Enter/exit thresholds for all Schmitt-trigger event latches."""

    visible_enter_distance: float = 6.0
    visible_exit_distance: float = 6.5
    visible_enter_half_fov_deg: float = 100.0
    visible_exit_half_fov_deg: float = 110.0
    near_enter_distance: float = 2.5
    near_exit_distance: float = 2.8
    personal_space_enter_distance: float = 1.0
    personal_space_exit_distance: float = 1.2
    fast_approach_enter_speed: float = 0.50
    fast_approach_exit_speed: float = 0.35
    ttc_low_enter_seconds: float = 1.5
    ttc_low_exit_seconds: float = 2.0
    leaving_enter_speed: float = -0.10
    leaving_exit_speed: float = 0.0
    sudden_near_min_closing_speed: float = 0.25
    sudden_near_max_closing_speed: float = 0.50

    def __post_init__(self) -> None:
        for item in fields(self):
            value = _finite_number(item.name, getattr(self, item.name))
            object.__setattr__(self, item.name, value)

        self._validate_band(
            "visible distance",
            self.visible_enter_distance,
            self.visible_exit_distance,
        )
        self._validate_band(
            "visible half FOV",
            self.visible_enter_half_fov_deg,
            self.visible_exit_half_fov_deg,
        )
        if not 0.0 < self.visible_enter_half_fov_deg <= 180.0:
            raise ValueError("visible_enter_half_fov_deg must be in (0, 180]")
        if not 0.0 < self.visible_exit_half_fov_deg <= 180.0:
            raise ValueError("visible_exit_half_fov_deg must be in (0, 180]")
        self._validate_band(
            "near distance", self.near_enter_distance, self.near_exit_distance
        )
        self._validate_band(
            "personal-space distance",
            self.personal_space_enter_distance,
            self.personal_space_exit_distance,
        )
        self._validate_reverse_band(
            "fast-approach speed",
            self.fast_approach_enter_speed,
            self.fast_approach_exit_speed,
        )
        self._validate_band(
            "low-TTC seconds",
            self.ttc_low_enter_seconds,
            self.ttc_low_exit_seconds,
        )
        if self.leaving_enter_speed >= self.leaving_exit_speed:
            raise ValueError("leaving_enter_speed must be below leaving_exit_speed")
        if self.leaving_exit_speed > 0.0:
            raise ValueError("leaving_exit_speed must be non-positive")
        if not (
            0.0
            <= self.sudden_near_min_closing_speed
            < self.sudden_near_max_closing_speed
            <= self.fast_approach_enter_speed
        ):
            raise ValueError(
                "sudden-near closing-speed band must be non-negative and end "
                "at or below fast_approach_enter_speed"
            )

    @staticmethod
    def _validate_band(name: str, enter: float, exit_: float) -> None:
        if enter < 0.0 or exit_ <= enter:
            raise ValueError(f"{name} requires 0 <= enter < exit")

    @staticmethod
    def _validate_reverse_band(name: str, enter: float, exit_: float) -> None:
        if exit_ < 0.0 or enter <= exit_:
            raise ValueError(f"{name} requires enter > exit >= 0")

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "EventThresholds":
        known = {item.name for item in fields(cls)}
        unknown = set(values) - known
        if unknown:
            names = ", ".join(sorted(str(key) for key in unknown))
            raise ValueError(f"unknown event threshold keys: {names}")
        return cls(**dict(values))


@dataclass(frozen=True)
class AutomatonTiming:
    attention_dwell_seconds: float = 0.5
    recovery_timeout_seconds: float = 3.0
    reentry_cooldown_seconds: float = 0.5

    def __post_init__(self) -> None:
        for item in fields(self):
            value = _finite_number(item.name, getattr(self, item.name))
            if value < 0.0:
                raise ValueError(f"{item.name} must be non-negative")
            object.__setattr__(self, item.name, value)

    @staticmethod
    def _seconds_to_ns(value: float) -> int:
        return int(round(value * 1_000_000_000.0))

    @property
    def attention_dwell_ns(self) -> int:
        return self._seconds_to_ns(self.attention_dwell_seconds)

    @property
    def recovery_timeout_ns(self) -> int:
        return self._seconds_to_ns(self.recovery_timeout_seconds)

    @property
    def reentry_cooldown_ns(self) -> int:
        return self._seconds_to_ns(self.reentry_cooldown_seconds)

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "AutomatonTiming":
        known = {item.name for item in fields(cls)}
        unknown = set(values) - known
        if unknown:
            names = ", ".join(sorted(str(key) for key in unknown))
            raise ValueError(f"unknown automaton timing keys: {names}")
        return cls(**dict(values))


@dataclass(frozen=True)
class FormalSocialConfig:
    """Complete core configuration for a one-human V1 instance."""

    target_agent_id: int = 1
    thresholds: EventThresholds = EventThresholds()
    timing: AutomatonTiming = AutomatonTiming()
    behavior_profiles: Mapping[FormalState, BehaviorProfile] = None  # type: ignore[assignment]

    def __post_init__(self) -> None:
        if not isinstance(self.target_agent_id, int) or isinstance(
            self.target_agent_id, bool
        ):
            raise TypeError("target_agent_id must be an integer")
        if self.target_agent_id < 0:
            raise ValueError("target_agent_id must be non-negative")
        if not isinstance(self.thresholds, EventThresholds):
            raise TypeError("thresholds must be EventThresholds")
        if not isinstance(self.timing, AutomatonTiming):
            raise TypeError("timing must be AutomatonTiming")

        raw_profiles = (
            default_behavior_profiles()
            if self.behavior_profiles is None
            else self.behavior_profiles
        )
        normalized = {
            FormalState(state): profile for state, profile in raw_profiles.items()
        }
        missing = set(FormalState) - set(normalized)
        if missing:
            names = ", ".join(sorted(state.value for state in missing))
            raise ValueError(f"behavior_profiles missing states: {names}")
        if any(not isinstance(profile, BehaviorProfile) for profile in normalized.values()):
            raise TypeError("all behavior_profiles values must be BehaviorProfile")
        object.__setattr__(
            self, "behavior_profiles", MappingProxyType(dict(normalized))
        )

    @classmethod
    def from_mapping(cls, document: Mapping[str, Any]) -> "FormalSocialConfig":
        """Parse either a direct mapping or a ROS-parameter YAML document.

        Supported nesting is ``formal_social_behavior.ros__parameters`` with
        ``thresholds``, ``timing``, and ``behavior_profiles`` sub-mappings.
        Flat dataclass keys are also accepted to keep launch-time dictionaries
        simple.
        """

        root: Mapping[str, Any] = document
        if "formal_social_behavior" in root:
            named_root = root["formal_social_behavior"]
            if not isinstance(named_root, Mapping):
                raise TypeError("formal_social_behavior must be a mapping")
            wrapper_siblings = set(root) - {"formal_social_behavior"}
            if wrapper_siblings:
                names = ", ".join(
                    sorted(str(key) for key in wrapper_siblings)
                )
                raise ValueError(
                    "unknown keys beside formal_social_behavior wrapper: "
                    f"{names}"
                )
            root = named_root
        if "ros__parameters" in root:
            ros_parameters = root["ros__parameters"]
            if not isinstance(ros_parameters, Mapping):
                raise TypeError("ros__parameters must be a mapping")
            wrapper_siblings = set(root) - {"ros__parameters"}
            if wrapper_siblings:
                names = ", ".join(
                    sorted(str(key) for key in wrapper_siblings)
                )
                raise ValueError(
                    "unknown keys beside ros__parameters wrapper: "
                    f"{names}"
                )
            root = ros_parameters

        threshold_keys = {item.name for item in fields(EventThresholds)}
        timing_keys = {item.name for item in fields(AutomatonTiming)}
        section_keys = {
            "target_agent_id",
            "thresholds",
            "timing",
            "behavior_profiles",
        }
        unknown_root = set(root) - section_keys - threshold_keys - timing_keys
        if unknown_root:
            names = ", ".join(sorted(str(key) for key in unknown_root))
            raise ValueError(f"unknown formal social config keys: {names}")

        flat_thresholds = threshold_keys.intersection(root)
        if "thresholds" in root and flat_thresholds:
            raise ValueError("thresholds cannot mix nested and flat keys")
        threshold_values = root.get(
            "thresholds",
            {key: root[key] for key in flat_thresholds},
        )

        flat_timing = timing_keys.intersection(root)
        if "timing" in root and flat_timing:
            raise ValueError("timing cannot mix nested and flat keys")
        timing_values = root.get(
            "timing",
            {key: root[key] for key in flat_timing},
        )
        if not isinstance(threshold_values, Mapping):
            raise TypeError("thresholds must be a mapping")
        if not isinstance(timing_values, Mapping):
            raise TypeError("timing must be a mapping")

        defaults = default_behavior_profiles()
        profile_values = root.get("behavior_profiles")
        profiles = dict(defaults)
        if profile_values is None:
            profile_values = {}
        if not isinstance(profile_values, Mapping):
            raise TypeError("behavior_profiles must be a mapping")
        if "behavior_profiles" in root:
            normalized_profile_states = set()
            for raw_state in profile_values:
                try:
                    normalized_state = FormalState(str(raw_state).upper())
                except ValueError as exc:
                    raise ValueError(
                        f"unknown formal state {raw_state!r}"
                    ) from exc
                if normalized_state in normalized_profile_states:
                    raise ValueError(
                        "duplicate behavior profile state after "
                        f"normalization: {normalized_state.value}"
                    )
                normalized_profile_states.add(normalized_state)
            missing_states = set(FormalState) - normalized_profile_states
            if missing_states:
                names = ", ".join(
                    sorted(state.value for state in missing_states)
                )
                raise ValueError(
                    f"behavior_profiles missing states: {names}"
                )
        for raw_state, raw_profile in profile_values.items():
            try:
                state = FormalState(str(raw_state).upper())
            except ValueError as exc:
                raise ValueError(f"unknown formal state {raw_state!r}") from exc
            if not isinstance(raw_profile, Mapping):
                raise TypeError(f"behavior profile {raw_state!r} must be a mapping")
            profiles[state] = BehaviorProfile.from_mapping(
                raw_profile, base=defaults[state]
            )

        return cls(
            target_agent_id=root.get("target_agent_id", 1),
            thresholds=EventThresholds.from_mapping(threshold_values),
            timing=AutomatonTiming.from_mapping(timing_values),
            behavior_profiles=profiles,
        )
