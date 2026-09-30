#!/usr/bin/env python3
"""Validate the generic one/six-person HuNav adapter inputs without Isaac."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import yaml

from arena_multi_hunav.adapter import HuNavMultiAdapter


ROOT = Path(__file__).resolve().parents[1]


class ParameterSource:
    def __init__(self, config: Path) -> None:
        self.config = str(config)

    def get_parameter(self, name: str):
        if name != "agent_config_file":
            raise KeyError(name)
        return SimpleNamespace(value=self.config)


def main() -> int:
    failures = []
    checks = 0
    profiles = {"regular.yaml": [1], "six_behaviors.yaml": [1, 2, 3, 4, 5, 6]}
    for filename, expected_types in profiles.items():
        path = ROOT / "config/hunav" / filename
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
        agents, templates = HuNavMultiAdapter._load_agents(ParameterSource(path))
        actual_types = sorted(agent.behavior.type for agent in agents.agents)
        node_parameters = document["arena_hunav_isaac_bridge"]["ros__parameters"]
        checks += 4
        if actual_types != expected_types:
            failures.append(f"{filename} behavior types={actual_types}")
        if len(templates) != len(expected_types):
            failures.append(f"{filename} template count")
        if len(node_parameters["character_models"]) != len(expected_types):
            failures.append(f"{filename} character model count")
        if any(agent.name not in templates for agent in agents.agents):
            failures.append(f"{filename} template names")
    result = {"passed": not failures, "checks": checks, "failures": failures}
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
