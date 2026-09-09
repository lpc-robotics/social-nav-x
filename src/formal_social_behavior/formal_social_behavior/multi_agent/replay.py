"""Verify committed schema-2 JSONL against the pure deterministic evaluator."""

import argparse
import hashlib
import json
from pathlib import Path

import yaml

from ..trace_serialization import json_text
from .automaton import MultiAgentAutomaton
from .config import MultiConfig
from .model_export import model_hash
from .trace import semantic_payload, snapshot_from_payload


def verify_replay(config_bytes, lines):
    config = MultiConfig.from_mapping(yaml.safe_load(config_bytes))
    config_hash = hashlib.sha256(config_bytes).hexdigest()
    engine, context, count, mode = None, None, 0, None
    for line in lines:
        frame = json.loads(line)
        shared = frame["shared_events_enabled"]
        if not isinstance(shared, bool):
            raise ValueError("shared_events_enabled must be boolean")
        if engine is None:
            mode = shared
            engine = MultiAgentAutomaton(config, shared_events_enabled=mode)
            context = engine.initial_context()
        if shared != mode or frame["commit_seq"] != count + 1:
            raise ValueError("mode change or missing/duplicated commit")
        snapshot = snapshot_from_payload(frame["input"])
        step = engine.evaluate(context, snapshot)
        if step.duplicate:
            raise ValueError("duplicate committed stamp")
        expected = semantic_payload(snapshot, step, config_sha256=config_hash,
            model_sha256=model_hash(config), names={a["agent_id"]: a["agent_name"] for a in frame["agents"]})
        for agent, memory in zip(expected["agents"], step.context.agents):
            agent["profile"] = config.behavior_profiles[memory.context.state].as_dict()
        for key, value in expected.items():
            if json_text(frame[key]) != json_text(value):
                raise ValueError(f"semantic mismatch at commit {count + 1}: {key}")
        context = step.context
        count += 1
    if not count:
        raise ValueError("empty trace")
    return count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config")
    parser.add_argument("steps")
    args = parser.parse_args()
    with Path(args.steps).open() as stream:
        count = verify_replay(Path(args.config).read_bytes(), stream)
    print(f"FORMAL_MULTI_REPLAY_OK commits={count}")


if __name__ == "__main__":
    main()
