"""Schema 2 batch telemetry and ROS-independent, replayable snapshots."""

from dataclasses import asdict
import math

from ..model import PlanarKinematics
from ..trace_serialization import json_text
from .model import SceneSnapshot


SCHEMA_VERSION = 2
MODEL_VERSION = "formal-social-pair-v1"


def snapshot_payload(snapshot):
    return {"sim_time_ns": snapshot.sim_time_ns, "robot": asdict(snapshot.robot),
            "humans": [{"agent_id": agent_id, **asdict(human)} for agent_id, human in snapshot.humans]}


def snapshot_from_payload(payload):
    humans = []
    for item in payload["humans"]:
        values = dict(item)
        agent_id = values.pop("agent_id")
        humans.append((agent_id, PlanarKinematics(**values)))
    return SceneSnapshot(payload["sim_time_ns"], PlanarKinematics(**payload["robot"]), tuple(humans))


def semantic_payload(snapshot, step, *, config_sha256, model_sha256, names=None):
    """Deterministic semantic data; wall time and backend retries live separately."""
    names = names or {}
    event_map = dict(step.robot_events)
    pair = step.pair_evaluation
    agents = []
    for memory in step.context.agents:
        event = event_map[memory.agent_id]
        metrics = asdict(event.metrics)
        if math.isinf(metrics["ttc_seconds"]):
            metrics["ttc_seconds"] = None
        agents.append({"agent_id": memory.agent_id,
                       "agent_name": names.get(memory.agent_id, str(memory.agent_id)),
                       "state": memory.context.state.value,
                       "context": asdict(memory.context),
                       "events": sorted(item.value for item in event.events),
                       "metrics": metrics})
        if pair:
            index = len(agents) - 1
            agents[-1]["peer_id"] = step.context.agents[1 - index].agent_id
            agents[-1]["peer_events"] = ["PEER_VISIBLE"] if pair.geometry.visible[index] else []
    return {"schema_version": SCHEMA_VERSION, "model_version": MODEL_VERSION,
            "message_type": "step", "epoch": step.context.epoch,
            "sim_time_ns": snapshot.sim_time_ns, "config_sha256": config_sha256,
            "model_sha256": model_sha256, "input": snapshot_payload(snapshot),
            "agents": agents,
            "pair": {"pair_id": [agent.agent_id for agent in step.context.agents],
                     "active": step.context.pair.active,
                     "session": step.context.pair.session,
                     "events": (["PEER_NEAR"] if pair and pair.geometry.near else []) +
                               (["MUTUAL_GAZE"] if pair and pair.geometry.gaze else []),
                     "memory": asdict(step.context.pair),
                     "metrics": asdict(pair.geometry) if pair else None},
            "shared_events": [asdict(event) for event in pair.shared_events] if pair else [],
            "transitions": [asdict(transition) for transition in step.transitions],
            "clock_reset": step.clock_reset}


def replay_frames(engine, snapshots, *, config_sha256="test", model_sha256="test"):
    context = engine.initial_context()
    frames = []
    for snapshot in snapshots:
        step = engine.evaluate(context, snapshot)
        if step.duplicate:
            continue
        frames.append(json_text(semantic_payload(snapshot, step, config_sha256=config_sha256,
                                                 model_sha256=model_sha256)))
        context = step.context
    return frames
