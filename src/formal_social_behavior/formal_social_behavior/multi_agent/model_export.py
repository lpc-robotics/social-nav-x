"""Finite model description for inspection and a later UPPAAL exporter.

This is not an UPPAAL proof or an executable guard-expression language.
"""

import argparse
from dataclasses import asdict
import hashlib
from pathlib import Path

import yaml

from ..automaton import transition_rules_for
from ..model import FormalState, SocialEvent
from ..trace_serialization import json_text
from .automaton import SOCIAL_RULES
from .config import MultiConfig
from .coordinator import PAIR_RULES
from .model import MultiState, PeerEvent
from .trace import MODEL_VERSION


def _ordered(rows):
    result = []
    for priority, row in enumerate(rows):
        result.append({**row, "priority": priority,
                       "effective_guard": {"guard": row["guard"],
                                           "exclude": [prior["guard"] for prior in rows[:priority]]}})
    return result


def describe_model(config: MultiConfig):
    transitions = {}
    low_guards = {"ROBOT_VISIBLE_AFTER_COOLDOWN", "ATTENTION_DWELL_NEAR_SAFE_CLOSING"}
    for state in MultiState:
        if state is MultiState.SOCIAL:
            rows = [asdict(rule) for rule in SOCIAL_RULES]
        else:
            rows = [{"rule_id": f"v1.{state.value.lower()}.{rule.cause.value.lower()}",
                     "guard": rule.guard.value, "destination": rule.destination.value,
                     "cause": rule.cause.value} for rule in transition_rules_for(FormalState(state.value))]
            if state in (MultiState.NORMAL, MultiState.ATTENTION):
                index = next(i for i, row in enumerate(rows) if row["guard"] in low_guards)
                rows.insert(index, {"rule_id": "social.form", "guard": "SOCIAL_SPACE_FORMED",
                                    "destination": "SOCIAL", "cause": "SOCIAL_SPACE_FORMED"})
        rows.append({"rule_id": f"{state.value.lower()}.hold", "guard": "TRUE",
                     "destination": state.value, "cause": None})
        transitions[state.value] = _ordered(rows)
    pair = {source: _ordered([asdict(rule) for rule in PAIR_RULES if rule.source == source] +
                [{"rule_id": f"pair.{source.lower()}.hold", "source": source, "guard": "TRUE",
                  "destination": source, "reason": None}])
            for source in ("INACTIVE", "ACTIVE")}
    return {"model_version": MODEL_VERSION, "agent_ids": config.agent_ids,
            "robot_events": [event.value for event in SocialEvent],
            "peer_events": [event.value for event in PeerEvent],
            "states": [state.value for state in MultiState], "initial_state": "NORMAL",
            "transitions": transitions, "pair_protocol": pair,
            "timing": asdict(config.timing), "robot_thresholds": asdict(config.thresholds),
            "peer_thresholds": asdict(config.peer),
            "profiles": {state.value: profile.as_dict() for state, profile in config.behavior_profiles.items()},
            "synchronization": {"protocol": "snapshot_then_atomic_commit", "participants": config.agent_ids,
                "formation_readiness": ["both NORMAL/ATTENTION", "cooldowns expired", "no danger/sudden/lost",
                    "peer near/gaze/stationary and disjoint disks", "robot outside both old and new space",
                    "continuous formation dwell"],
                "social_invariant": "pair.active iff both agents are SOCIAL at commit boundaries",
                "independent_local_mode": "disable pair protocol and social.form rows"},
            "clocks": {"unit": "integer nanoseconds", "source": "ComputeAgents input simulation stamp",
                "state_entered": "reset on every state transition",
                "safe_since": "clear on danger; SURPRISED/SCARED start a fresh residence recovery timer",
                "cooldown_until": "set on entry to NORMAL",
                "pair_clocks": ["ready_since", "broken_since", "cooldown_until"],
                "time_reset": "higher priority than all ordinary transitions; clear both contexts and pair; new epoch"},
            "assumptions": ["exactly two fixed humans", "no occlusion", "linear sweep between observations",
                "HuNav/SFM continuous motion is an environment input, not a symbolic proof",
                "recovery liveness requires danger clearance, leaving/lost guards and progressing time",
                "backend batch reset restarts both BT clocks, never formal clocks"],
            "uppaal_exported": False}


def model_hash(config):
    return hashlib.sha256(json_text(describe_model(config)).encode()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config")
    parser.add_argument("--output")
    args = parser.parse_args()
    config = MultiConfig.from_mapping(yaml.safe_load(Path(args.config).read_text()))
    text = json_text(describe_model(config)) + "\n"
    if args.output:
        Path(args.output).write_text(text, encoding="utf-8")
    else:
        print(text, end="")
