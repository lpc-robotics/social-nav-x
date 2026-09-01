"""Deterministic V1 formal social-state automaton."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
from typing import FrozenSet, Tuple

from .config import AutomatonTiming
from .model import (
    AutomatonContext,
    AutomatonStep,
    EventSnapshot,
    FormalState,
    SocialEvent,
    Transition,
    TransitionCause,
)


class GuardName(str, Enum):
    """Finite guard vocabulary; V1 intentionally has no guard DSL."""

    PERSONAL_SPACE_VIOLATION = "PERSONAL_SPACE_VIOLATION"
    TTC_LOW = "TTC_LOW"
    ROBOT_FAST_APPROACH = "ROBOT_FAST_APPROACH"
    SUDDEN_NEAR = "SUDDEN_NEAR"
    ROBOT_VISIBLE_AFTER_COOLDOWN = "ROBOT_VISIBLE_AFTER_COOLDOWN"
    ROBOT_LOST = "ROBOT_LOST"
    SCARED_ROBOT_LOST_WHEN_SAFE = "SCARED_ROBOT_LOST_WHEN_SAFE"
    ATTENTION_DWELL_NEAR_SAFE_CLOSING = "ATTENTION_DWELL_NEAR_SAFE_CLOSING"
    LEAVING_OUTSIDE_NEAR = "LEAVING_OUTSIDE_NEAR"
    SAFE_RECOVERY_TIMEOUT = "SAFE_RECOVERY_TIMEOUT"
    LEAVING_OUTSIDE_NEAR_AFTER_SAFE_TIMEOUT = (
        "LEAVING_OUTSIDE_NEAR_AFTER_SAFE_TIMEOUT"
    )


@dataclass(frozen=True)
class TransitionRule:
    """An ordered transition-table row."""

    source_states: FrozenSet[FormalState]
    guard: GuardName
    destination: FormalState
    cause: TransitionCause


_DANGER_SOURCES = frozenset(
    {
        FormalState.NORMAL,
        FormalState.ATTENTION,
        FormalState.CURIOUS,
        FormalState.SURPRISED,
    }
)


# Tuple order is the deterministic priority order.  The three safety guards are
# intentionally first and use the priority documented by the project.
TRANSITION_RULES: Tuple[TransitionRule, ...] = (
    TransitionRule(
        _DANGER_SOURCES,
        GuardName.PERSONAL_SPACE_VIOLATION,
        FormalState.SCARED,
        TransitionCause.PERSONAL_SPACE_VIOLATION,
    ),
    TransitionRule(
        _DANGER_SOURCES,
        GuardName.TTC_LOW,
        FormalState.SCARED,
        TransitionCause.TTC_LOW,
    ),
    TransitionRule(
        _DANGER_SOURCES,
        GuardName.ROBOT_FAST_APPROACH,
        FormalState.SCARED,
        TransitionCause.ROBOT_FAST_APPROACH,
    ),
    TransitionRule(
        frozenset({FormalState.NORMAL}),
        GuardName.SUDDEN_NEAR,
        FormalState.SURPRISED,
        TransitionCause.SUDDEN_NEAR,
    ),
    TransitionRule(
        frozenset({FormalState.NORMAL}),
        GuardName.ROBOT_VISIBLE_AFTER_COOLDOWN,
        FormalState.ATTENTION,
        TransitionCause.ROBOT_VISIBLE,
    ),
    TransitionRule(
        frozenset({FormalState.ATTENTION}),
        GuardName.ROBOT_LOST,
        FormalState.NORMAL,
        TransitionCause.ROBOT_LOST,
    ),
    TransitionRule(
        frozenset({FormalState.ATTENTION}),
        GuardName.SUDDEN_NEAR,
        FormalState.SURPRISED,
        TransitionCause.SUDDEN_NEAR,
    ),
    TransitionRule(
        frozenset({FormalState.ATTENTION}),
        GuardName.ATTENTION_DWELL_NEAR_SAFE_CLOSING,
        FormalState.CURIOUS,
        TransitionCause.ATTENTION_DWELL,
    ),
    TransitionRule(
        frozenset({FormalState.CURIOUS}),
        GuardName.ROBOT_LOST,
        FormalState.NORMAL,
        TransitionCause.ROBOT_LOST,
    ),
    TransitionRule(
        frozenset({FormalState.CURIOUS}),
        GuardName.LEAVING_OUTSIDE_NEAR,
        FormalState.NORMAL,
        TransitionCause.ROBOT_LEAVING,
    ),
    TransitionRule(
        frozenset({FormalState.SURPRISED}),
        GuardName.ROBOT_LOST,
        FormalState.NORMAL,
        TransitionCause.ROBOT_LOST,
    ),
    TransitionRule(
        frozenset({FormalState.SURPRISED}),
        GuardName.SAFE_RECOVERY_TIMEOUT,
        FormalState.NORMAL,
        TransitionCause.RECOVERY_TIMEOUT,
    ),
    TransitionRule(
        frozenset({FormalState.SCARED}),
        GuardName.SCARED_ROBOT_LOST_WHEN_SAFE,
        FormalState.NORMAL,
        TransitionCause.ROBOT_LOST,
    ),
    TransitionRule(
        frozenset({FormalState.SCARED}),
        GuardName.LEAVING_OUTSIDE_NEAR_AFTER_SAFE_TIMEOUT,
        FormalState.NORMAL,
        TransitionCause.RECOVERY_TIMEOUT,
    ),
)


_DANGER_EVENTS = frozenset(
    {
        SocialEvent.PERSONAL_SPACE_VIOLATION,
        SocialEvent.TTC_LOW,
        SocialEvent.ROBOT_FAST_APPROACH,
    }
)


def transition_rules_for(state: FormalState) -> Tuple[TransitionRule, ...]:
    """Enumerate ordered rows applicable to ``state``."""

    normalized = FormalState(state)
    return tuple(rule for rule in TRANSITION_RULES if normalized in rule.source_states)


class SocialAutomaton:
    """Stateless evaluator whose caller commits the returned context."""

    def __init__(self, timing: AutomatonTiming | None = None) -> None:
        self.timing = timing or AutomatonTiming()

    @staticmethod
    def initial_context(*, last_stamp_ns: int | None = None) -> AutomatonContext:
        return AutomatonContext(last_stamp_ns=last_stamp_ns)

    def step(
        self,
        context: AutomatonContext,
        events: EventSnapshot,
    ) -> AutomatonStep:
        stamp = events.sim_time_ns
        clock_reset = (
            events.clock_rollback
            or SocialEvent.TIME_RESET in events.events
            or (context.last_stamp_ns is not None and stamp < context.last_stamp_ns)
        )
        if clock_reset:
            transition = None
            reset_context = self.initial_context(last_stamp_ns=stamp)
            if context.state is not FormalState.NORMAL:
                transition = Transition(
                    sim_time_ns=stamp,
                    old_state=context.state,
                    new_state=FormalState.NORMAL,
                    cause=TransitionCause.TIME_RESET,
                )
                reset_context = replace(
                    reset_context, last_transition_stamp_ns=stamp
                )
            return AutomatonStep(
                context=reset_context,
                event_snapshot=events,
                transition=transition,
                clock_reset=True,
            )

        danger_active = bool(events.events & _DANGER_EVENTS)
        safe_since_ns = context.safe_since_ns
        if danger_active:
            safe_since_ns = None
        elif safe_since_ns is None:
            safe_since_ns = stamp

        state_entered_ns = context.state_entered_ns
        if state_entered_ns is None:
            state_entered_ns = stamp
        cooldown_until_ns = context.cooldown_until_ns
        if (
            context.state is FormalState.NORMAL
            and cooldown_until_ns is not None
            and stamp >= cooldown_until_ns
        ):
            cooldown_until_ns = None

        candidate_context = replace(
            context,
            state_entered_ns=state_entered_ns,
            safe_since_ns=safe_since_ns,
            cooldown_until_ns=cooldown_until_ns,
            last_stamp_ns=stamp,
        )

        # A retried or duplicate observation may update latches, but it cannot
        # produce a second transition at the same simulation timestamp.
        if context.last_transition_stamp_ns == stamp:
            return AutomatonStep(
                context=candidate_context,
                event_snapshot=events,
            )

        for rule in transition_rules_for(context.state):
            if not self._guard_matches(
                rule.guard, candidate_context, events, danger_active
            ):
                continue
            transition = Transition(
                sim_time_ns=stamp,
                old_state=context.state,
                new_state=rule.destination,
                cause=rule.cause,
            )
            next_safe_since = candidate_context.safe_since_ns
            if rule.destination is FormalState.SURPRISED:
                next_safe_since = stamp
            elif rule.destination is FormalState.SCARED:
                next_safe_since = None if danger_active else stamp
            elif rule.destination is FormalState.NORMAL:
                next_safe_since = None if danger_active else stamp

            next_cooldown = None
            if rule.destination is FormalState.NORMAL:
                next_cooldown = stamp + self.timing.reentry_cooldown_ns

            next_context = replace(
                candidate_context,
                state=rule.destination,
                state_entered_ns=stamp,
                safe_since_ns=next_safe_since,
                cooldown_until_ns=next_cooldown,
                last_transition_stamp_ns=stamp,
            )
            return AutomatonStep(
                context=next_context,
                event_snapshot=events,
                transition=transition,
            )

        return AutomatonStep(
            context=candidate_context,
            event_snapshot=events,
        )

    def _guard_matches(
        self,
        guard: GuardName,
        context: AutomatonContext,
        events: EventSnapshot,
        danger_active: bool,
    ) -> bool:
        active = events.events
        stamp = events.sim_time_ns
        if guard is GuardName.PERSONAL_SPACE_VIOLATION:
            return SocialEvent.PERSONAL_SPACE_VIOLATION in active
        if guard is GuardName.TTC_LOW:
            return SocialEvent.TTC_LOW in active
        if guard is GuardName.ROBOT_FAST_APPROACH:
            return SocialEvent.ROBOT_FAST_APPROACH in active
        if guard is GuardName.SUDDEN_NEAR:
            return SocialEvent.SUDDEN_NEAR in active
        if guard is GuardName.ROBOT_VISIBLE_AFTER_COOLDOWN:
            return (
                SocialEvent.ROBOT_VISIBLE in active
                and (
                    context.cooldown_until_ns is None
                    or stamp >= context.cooldown_until_ns
                )
            )
        if guard is GuardName.ROBOT_LOST:
            return SocialEvent.ROBOT_LOST in active
        if guard is GuardName.SCARED_ROBOT_LOST_WHEN_SAFE:
            # Scared turns the body away from the robot as part of its escape
            # response. That can create ROBOT_LOST on the first post-reset
            # observation even while the dangerous approach is still active.
            # Safety has priority: remain Scared on that beat instead of
            # rebuilding Regular and immediately escalating again.
            return SocialEvent.ROBOT_LOST in active and not danger_active
        if guard is GuardName.ATTENTION_DWELL_NEAR_SAFE_CLOSING:
            dwell_elapsed = (
                context.state_entered_ns is not None
                and stamp - context.state_entered_ns
                >= self.timing.attention_dwell_ns
            )
            return (
                dwell_elapsed
                and SocialEvent.ROBOT_SAFE_APPROACH in active
                and not danger_active
            )
        if guard is GuardName.LEAVING_OUTSIDE_NEAR:
            return (
                SocialEvent.ROBOT_LEAVING in active
                and SocialEvent.ROBOT_NEAR not in active
            )
        if guard is GuardName.SAFE_RECOVERY_TIMEOUT:
            return (
                not danger_active
                and context.safe_since_ns is not None
                and stamp - context.safe_since_ns
                >= self.timing.recovery_timeout_ns
            )
        if guard is GuardName.LEAVING_OUTSIDE_NEAR_AFTER_SAFE_TIMEOUT:
            return (
                SocialEvent.ROBOT_LEAVING in active
                and SocialEvent.ROBOT_NEAR not in active
                and not danger_active
                and context.safe_since_ns is not None
                and stamp - context.safe_since_ns
                >= self.timing.recovery_timeout_ns
            )
        raise AssertionError(f"unhandled guard {guard.value}")
