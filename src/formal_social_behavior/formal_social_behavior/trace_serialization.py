"""Deterministic JSON payloads for formal social observations and traces."""

from __future__ import annotations

import json
import math
from typing import Any, Mapping

from .model import EventSnapshot, FormalState, Transition


SCHEMA_VERSION = 1


def _json_float(value: float) -> float | None:
    return float(value) if math.isfinite(value) else None


def json_text(payload: Mapping[str, Any]) -> str:
    """Serialize without NaN and with stable key ordering/separators."""

    return json.dumps(
        payload,
        allow_nan=False,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    )


def observation_payload(
    *,
    event_snapshot: EventSnapshot,
    state: FormalState,
    behavior_type: int,
    agent_id: int,
    agent_name: str,
    config_sha256: str,
    reset_count: int,
) -> dict[str, Any]:
    """Build the shared state/event payload required by schema V1."""

    metrics = event_snapshot.metrics
    return {
        "schema_version": SCHEMA_VERSION,
        "sim_time_ns": event_snapshot.sim_time_ns,
        "agent_id": int(agent_id),
        "agent_name": str(agent_name),
        "state": FormalState(state).value,
        "events": sorted(event.value for event in event_snapshot.events),
        "distance": metrics.distance,
        "closing_speed": metrics.closing_speed,
        "ttc_seconds": _json_float(metrics.ttc_seconds),
        "bearing_to_robot": metrics.bearing_to_robot,
        "relative_bearing": metrics.relative_bearing,
        "behavior_type": int(behavior_type),
        "reset_count": int(reset_count),
        "config_sha256": str(config_sha256),
    }


def transition_payload(
    observation: Mapping[str, Any], transition: Transition
) -> dict[str, Any]:
    """Extend a committed observation with transition-specific fields."""

    return {
        **dict(observation),
        "message_type": "transition",
        "old_state": transition.old_state.value,
        "new_state": transition.new_state.value,
        "cause": transition.cause.value,
    }
