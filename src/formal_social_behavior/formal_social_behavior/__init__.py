"""Deterministic one-robot/one-human social automaton for social-nav-x."""

from .automaton import (
    TRANSITION_RULES,
    GuardName,
    SocialAutomaton,
    TransitionRule,
    transition_rules_for,
)
from .behavior_adapter import (
    BehaviorProfile,
    BehaviorType,
    apply_profile_to_agent,
    default_behavior_profiles,
    profile_for_state,
    profile_signature,
)
from .config import AutomatonTiming, EventThresholds, FormalSocialConfig
from .event_extractor import (
    EventExtractor,
    disk_ttc_seconds,
    interaction_metrics,
    normalize_angle,
)
from .model import (
    AutomatonContext,
    AutomatonStep,
    EventEvaluation,
    EventMemory,
    EventSnapshot,
    FormalState,
    InteractionMetrics,
    MotionSnapshot,
    PlanarKinematics,
    SocialEvent,
    Transition,
    TransitionCause,
)

__all__ = [
    "AutomatonContext",
    "AutomatonStep",
    "AutomatonTiming",
    "BehaviorProfile",
    "BehaviorType",
    "EventEvaluation",
    "EventExtractor",
    "EventMemory",
    "EventSnapshot",
    "EventThresholds",
    "FormalSocialConfig",
    "FormalState",
    "GuardName",
    "InteractionMetrics",
    "MotionSnapshot",
    "PlanarKinematics",
    "SocialAutomaton",
    "SocialEvent",
    "TRANSITION_RULES",
    "Transition",
    "TransitionCause",
    "TransitionRule",
    "apply_profile_to_agent",
    "default_behavior_profiles",
    "disk_ttc_seconds",
    "interaction_metrics",
    "normalize_angle",
    "profile_for_state",
    "profile_signature",
    "transition_rules_for",
]
