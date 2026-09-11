#!/usr/bin/env python3
"""Generate fully expanded HuNav fixtures accepted by the installed ROS parser."""

import copy
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = ROOT / "src/arena_mpc_bringup/config/hunav"
BASE_PATH = CONFIG_DIR / "p4_crossing.yaml"


def parameters(document):
    return document["hunav_loader"]["ros__parameters"]


def route(spec, *, xyh, goals, speed, cyclic=False):
    old_goal_names = list(spec["goals"])
    spec["max_vel"] = speed
    spec["behavior"]["vel"] = speed
    spec["init_pose"] = {"x": xyh[0], "y": xyh[1], "z": 0.0, "h": xyh[2]}
    spec["cyclic_goals"] = cyclic
    spec["goals"] = [name for name, _ in goals]
    for key in old_goal_names:
        spec.pop(key, None)
    for name, (x, y) in goals:
        spec[name] = {"x": x, "y": y, "h": 0.0}


def scenario(base, name):
    document = copy.deepcopy(base)
    parameters(document)["map"] = f"p4_{name}"
    return document


def write(document, name):
    path = CONFIG_DIR / f"p4_{name}.yaml"
    path.write_text(
        yaml.safe_dump(document, sort_keys=False, default_flow_style=False),
        encoding="utf-8",
    )


def main():
    base = yaml.safe_load(BASE_PATH.read_text(encoding="utf-8"))

    head_on = scenario(base, "head_on")
    route(
        parameters(head_on)["crossing"],
        xyh=(8.2, 4.0, 3.1415926536),
        goals=[("p4_goal_west", (2.0, 4.0))],
        speed=0.25,
    )
    write(head_on, "head_on")

    same_direction = scenario(base, "same_direction")
    route(
        parameters(same_direction)["crossing"],
        xyh=(4.2, 4.0, 0.0),
        goals=[("p4_goal_east", (9.5, 4.0))],
        speed=0.18,
    )
    write(same_direction, "same_direction")

    multi_crossing = scenario(base, "multi_crossing")
    route(
        parameters(multi_crossing)["crossing"],
        xyh=(5.2, 1.0, 1.5707963268),
        goals=[("p4_goal_north", (5.2, 5.0))],
        speed=0.25,
    )
    route(
        parameters(multi_crossing)["filler_impassive"],
        xyh=(6.8, 5.0, -1.5707963268),
        goals=[("p4_goal_south", (6.8, 1.0))],
        speed=0.25,
    )
    write(multi_crossing, "multi_crossing")

    stop_turn = scenario(base, "stop_turn")
    route(
        parameters(stop_turn)["crossing"],
        xyh=(20.0, 15.0, 0.0),
        goals=[("p4_goal_east", (22.0, 15.0)), ("p4_goal_west", (20.0, 15.0))],
        speed=0.2,
        cyclic=True,
    )
    surprised = parameters(stop_turn)["filler_surprised"]
    surprised["behavior"]["dist"] = 2.5
    surprised["behavior"]["duration"] = 4.0
    surprised["behavior"]["once"] = True
    route(
        surprised,
        xyh=(6.0, 4.0, 3.1415926536),
        goals=[("p4_goal_west", (2.0, 4.0))],
        speed=0.22,
    )
    scared = parameters(stop_turn)["filler_scared"]
    scared["behavior"]["dist"] = 3.0
    scared["behavior"]["duration"] = 5.0
    scared["behavior"]["once"] = True
    route(
        scared,
        xyh=(6.5, 2.0, 3.1415926536),
        goals=[("p4_goal_west", (2.5, 2.0))],
        speed=0.25,
    )
    write(stop_turn, "stop_turn")

    write(scenario(base, "id_change"), "id_change")
    write(scenario(base, "backlog"), "backlog")


if __name__ == "__main__":
    main()
