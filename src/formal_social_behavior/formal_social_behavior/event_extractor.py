"""Convert continuous map-plane motion into deterministic social events."""

from __future__ import annotations

import math
from typing import Iterable

from .config import EventThresholds
from .model import (
    EventEvaluation,
    EventMemory,
    EventSnapshot,
    InteractionMetrics,
    MotionSnapshot,
    SocialEvent,
)


_EPSILON = 1.0e-12


def normalize_angle(angle: float) -> float:
    """Normalize radians to the closed-open interval ``[-pi, pi)``."""

    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def disk_ttc_seconds(
    relative_x: float,
    relative_y: float,
    relative_vx: float,
    relative_vy: float,
    combined_radius: float,
) -> float:
    """Return the first non-negative collision time for two moving disks.

    Relative quantities are robot minus human.  The calculation solves
    ``|r + v*t| = combined_radius`` under constant planar velocity.
    """

    if combined_radius < 0.0 or not math.isfinite(combined_radius):
        raise ValueError("combined_radius must be finite and non-negative")
    distance_squared = relative_x * relative_x + relative_y * relative_y
    radius_squared = combined_radius * combined_radius
    if distance_squared <= radius_squared:
        return 0.0

    a = relative_vx * relative_vx + relative_vy * relative_vy
    if a <= _EPSILON:
        return math.inf
    b = 2.0 * (relative_x * relative_vx + relative_y * relative_vy)
    c = distance_squared - radius_squared
    discriminant = b * b - 4.0 * a * c
    if discriminant < 0.0:
        return math.inf

    root = math.sqrt(max(0.0, discriminant))
    denominator = 2.0 * a
    roots: Iterable[float] = ((-b - root) / denominator, (-b + root) / denominator)
    non_negative = [value for value in roots if value >= 0.0]
    return min(non_negative) if non_negative else math.inf


def interaction_metrics(snapshot: MotionSnapshot) -> InteractionMetrics:
    """Calculate distance, closing speed, bearing, and disk TTC."""

    relative_x = snapshot.robot.x - snapshot.human.x
    relative_y = snapshot.robot.y - snapshot.human.y
    relative_vx = snapshot.robot.vx - snapshot.human.vx
    relative_vy = snapshot.robot.vy - snapshot.human.vy
    distance = math.hypot(relative_x, relative_y)
    if distance <= _EPSILON:
        closing_speed = 0.0
        bearing = snapshot.human.yaw
    else:
        closing_speed = -(
            relative_x * relative_vx + relative_y * relative_vy
        ) / distance
        bearing = math.atan2(relative_y, relative_x)
    relative_bearing = normalize_angle(bearing - snapshot.human.yaw)
    ttc_seconds = disk_ttc_seconds(
        relative_x,
        relative_y,
        relative_vx,
        relative_vy,
        snapshot.robot.radius + snapshot.human.radius,
    )
    return InteractionMetrics(
        distance=distance,
        closing_speed=closing_speed,
        ttc_seconds=ttc_seconds,
        bearing_to_robot=bearing,
        relative_bearing=relative_bearing,
    )


def _low_value_latch(
    value: float,
    previous: bool,
    enter_threshold: float,
    exit_threshold: float,
) -> bool:
    if previous:
        return value < exit_threshold
    return value <= enter_threshold


def _high_value_latch(
    value: float,
    previous: bool,
    enter_threshold: float,
    exit_threshold: float,
) -> bool:
    if previous:
        return value > exit_threshold
    return value >= enter_threshold


class EventExtractor:
    """Stateless evaluator whose caller owns the returned latch memory."""

    def __init__(self, thresholds: EventThresholds | None = None) -> None:
        self.thresholds = thresholds or EventThresholds()

    def evaluate(
        self,
        snapshot: MotionSnapshot,
        memory: EventMemory | None = None,
    ) -> EventEvaluation:
        previous = memory or EventMemory()
        metrics = interaction_metrics(snapshot)

        if (
            previous.last_stamp_ns is not None
            and snapshot.sim_time_ns < previous.last_stamp_ns
        ):
            reset_memory = EventMemory(last_stamp_ns=snapshot.sim_time_ns)
            reset_events = EventSnapshot(
                sim_time_ns=snapshot.sim_time_ns,
                events=frozenset({SocialEvent.TIME_RESET}),
                metrics=metrics,
                clock_rollback=True,
            )
            return EventEvaluation(event_snapshot=reset_events, memory=reset_memory)

        thresholds = self.thresholds
        relative_bearing_deg = abs(math.degrees(metrics.relative_bearing))
        if previous.robot_visible:
            robot_visible = (
                metrics.distance < thresholds.visible_exit_distance
                and relative_bearing_deg < thresholds.visible_exit_half_fov_deg
            )
        else:
            robot_visible = (
                metrics.distance <= thresholds.visible_enter_distance
                and relative_bearing_deg <= thresholds.visible_enter_half_fov_deg
            )

        robot_near = _low_value_latch(
            metrics.distance,
            previous.robot_near,
            thresholds.near_enter_distance,
            thresholds.near_exit_distance,
        )
        personal_space_violation = _low_value_latch(
            metrics.distance,
            previous.personal_space_violation,
            thresholds.personal_space_enter_distance,
            thresholds.personal_space_exit_distance,
        )
        robot_fast_approach = _high_value_latch(
            metrics.closing_speed,
            previous.robot_fast_approach,
            thresholds.fast_approach_enter_speed,
            thresholds.fast_approach_exit_speed,
        )
        ttc_low = _low_value_latch(
            metrics.ttc_seconds,
            previous.ttc_low,
            thresholds.ttc_low_enter_seconds,
            thresholds.ttc_low_exit_seconds,
        )

        if previous.robot_leaving:
            robot_leaving = metrics.closing_speed < thresholds.leaving_exit_speed
        else:
            robot_leaving = metrics.closing_speed <= thresholds.leaving_enter_speed

        sudden_near = (
            robot_near
            and not previous.robot_near
            and thresholds.sudden_near_min_closing_speed
            <= metrics.closing_speed
            < thresholds.sudden_near_max_closing_speed
            and not personal_space_violation
            and not ttc_low
            and not robot_fast_approach
        )
        robot_safe_approach = (
            robot_near
            and 0.0
            <= metrics.closing_speed
            < thresholds.sudden_near_min_closing_speed
            and not personal_space_violation
            and not ttc_low
            and not robot_fast_approach
        )

        active: set[SocialEvent] = set()
        if robot_visible:
            active.add(SocialEvent.ROBOT_VISIBLE)
        elif previous.robot_visible:
            active.add(SocialEvent.ROBOT_LOST)
        if robot_near:
            active.add(SocialEvent.ROBOT_NEAR)
        if robot_safe_approach:
            active.add(SocialEvent.ROBOT_SAFE_APPROACH)
        if sudden_near:
            active.add(SocialEvent.SUDDEN_NEAR)
        if personal_space_violation:
            active.add(SocialEvent.PERSONAL_SPACE_VIOLATION)
        if robot_fast_approach:
            active.add(SocialEvent.ROBOT_FAST_APPROACH)
        if ttc_low:
            active.add(SocialEvent.TTC_LOW)
        if robot_leaving:
            active.add(SocialEvent.ROBOT_LEAVING)

        next_memory = EventMemory(
            robot_visible=robot_visible,
            robot_near=robot_near,
            personal_space_violation=personal_space_violation,
            robot_fast_approach=robot_fast_approach,
            ttc_low=ttc_low,
            robot_leaving=robot_leaving,
            last_stamp_ns=snapshot.sim_time_ns,
        )
        event_snapshot = EventSnapshot(
            sim_time_ns=snapshot.sim_time_ns,
            events=frozenset(active),
            metrics=metrics,
        )
        return EventEvaluation(event_snapshot=event_snapshot, memory=next_memory)
