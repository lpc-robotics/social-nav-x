#!/usr/bin/env python3
"""Offline acceptance checks for scenario, interface, map, URDF, and Nav2 contracts."""

from __future__ import annotations

import json
import math
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/arena_multi_control"))
from arena_multi_control.scenario import load_scenario  # noqa: E402


def main() -> int:
    failures = []
    checks = 0

    def check(name: str, condition: bool) -> None:
        nonlocal checks
        checks += 1
        if not condition:
            failures.append(name)

    scenarios = sorted((ROOT / "config/scenarios").glob("*.yaml"))
    parsed = []
    for path in scenarios:
        try:
            scenario = load_scenario(path)
            parsed.append(scenario)
            check(f"scenario:{path.name}", True)
        except Exception:
            check(f"scenario:{path.name}", False)

    nav2 = yaml.safe_load((ROOT / "src/arena_multi_bringup/config/nav2_multirobot.yaml").read_text())
    control = nav2["controller_server"]["ros__parameters"]["FollowPath"]
    local = nav2["local_costmap"]["local_costmap"]["ros__parameters"]
    normalized = local["voxel_layer"]["normalized"]
    check("legacy dwb linear limit", control["max_vel_x"] == 1.0 and control["max_speed_xy"] == 1.0)
    check("legacy dwb angular limit", control["max_vel_theta"] == 1.2)
    check("normalized observation heights", normalized["min_obstacle_height"] == 0.0 and normalized["max_obstacle_height"] == 2.0)
    check("DWB usable angular sample grid", control["vtheta_samples"] == 4 and control["min_speed_theta"] == 0.30)
    check("footprint critic", "ObstacleFootprint" in control["critics"])
    check("normalized scan only", local["voxel_layer"]["observation_sources"] == "normalized")
    check("REP-117 no-return clearing", normalized["inf_is_valid"] is True)
    check("peer truth layer", local["peer_layer"]["plugin"] == "arena_peer_costmap/PeerObstacleLayer")
    bt = nav2["bt_navigator"]["ros__parameters"]
    check("scaled Nav2 server timeout", bt["default_server_timeout"] >= 500)
    stable_bt = ROOT / "src/arena_multi_bringup/behavior_trees/navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml"
    stable_bt_text = stable_bt.read_text()
    check("versioned stable-path BT", stable_bt.is_file() and "IsPathValid" in stable_bt_text)
    launch_text = (ROOT / "src/arena_multi_bringup/launch/multirobot.launch.py").read_text()
    check("stable-path BT selected", "default_nav_to_pose_bt_xml" in launch_text)

    urdf = ET.parse(ROOT / "config/urdf/jackal.urdf").getroot()
    chassis = next(link for link in urdf.findall("link") if link.attrib.get("name") == "chassis_link")
    size = [float(value) for value in chassis.find("collision/geometry/box").attrib["size"].split()]
    check("footprint contains chassis", 0.48 >= size[0] and 0.44 >= size[1])

    image = (ROOT / "config/maps/map.png").read_bytes()
    width, height = struct.unpack(">II", image[16:24])
    map_yaml = yaml.safe_load((ROOT / "config/maps/map.yaml").read_text())
    world = yaml.safe_load((ROOT / "config/world/arena.yaml").read_text())
    resolution = float(map_yaml["resolution"])
    expected_width = round((float(world["width"]) + 2.0 * float(world["map_margin"])) / resolution) + 1
    expected_height = round((float(world["height"]) + 2.0 * float(world["map_margin"])) / resolution) + 1
    check("map generated from world dimensions", width == expected_width and height == expected_height)
    check("map/world resolution", resolution == float(world["map_resolution"]))
    check("map/world origin", map_yaml["origin"][:2] == [-float(world["map_margin"])] * 2)

    for scenario in parsed:
        names = [robot.name for robot in scenario.robots]
        check("unique topics:" + ",".join(names), len(names) == len(set(names)))
        for robot in scenario.robots:
            check(
                f"spawn inside walls:{robot.name}@{len(names)}",
                0.5 <= robot.x <= float(world["width"]) - 0.5
                and 0.5 <= robot.y <= float(world["height"]) - 0.5,
            )

    isaac = (ROOT / "src/arena_isaac/arena_isaac/run_isaacsim.py").read_text()
    check("per-robot D6 state", "self._ideal_states" in isaac and "ARENA_ROBOT_NAMES" in isaac)
    check("normalized scan update", "NormalizedScan.update_all" in isaac)
    result = {"passed": not failures, "checks": checks, "failures": failures}
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
