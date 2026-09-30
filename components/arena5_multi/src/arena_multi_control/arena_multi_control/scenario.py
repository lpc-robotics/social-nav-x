"""Strict scenario parsing shared by launch files, nodes, and tests."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class Robot:
    name: str
    x: float
    y: float
    yaw: float
    control_mode: str


@dataclass(frozen=True)
class Scenario:
    robots: tuple[Robot, ...]
    planner_algorithm: str
    hunav_profile: str
    random_seed: int
    pedestrian_backend: str = "legacy_hunav"
    pedestrian_config: str = ""
    psychology_model: str = "noop"
    max_linear: float = 1.0
    max_angular: float = 1.2
    physics_dt: float = 1.0 / 60.0


def _finite_number(value: Any, field: str) -> float:
    import math

    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} must be numeric") from exc
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
    return result


def load_scenario(path: str | Path) -> Scenario:
    document = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("scenario root must be a mapping")
    raw_robots = document.get("robots")
    if not isinstance(raw_robots, list) or not raw_robots:
        raise ValueError("scenario must contain at least one robot")
    robots = []
    names = set()
    for index, raw in enumerate(raw_robots):
        if not isinstance(raw, dict):
            raise ValueError(f"robots[{index}] must be a mapping")
        name = str(raw.get("name", ""))
        if not name.startswith("robot_") or not name[6:].isdigit():
            raise ValueError(f"invalid robot name: {name!r}")
        if name in names:
            raise ValueError(f"duplicate robot name: {name}")
        names.add(name)
        mode = str(raw.get("control_mode", "nav2"))
        if mode not in ("nav2", "external"):
            raise ValueError(f"invalid control mode for {name}: {mode}")
        robots.append(
            Robot(
                name=name,
                x=_finite_number(raw.get("x"), f"{name}.x"),
                y=_finite_number(raw.get("y"), f"{name}.y"),
                yaw=_finite_number(raw.get("yaw", 0.0), f"{name}.yaw"),
                control_mode=mode,
            )
        )
    algorithm = str(document.get("planner_algorithm", "dijkstra")).lower()
    if algorithm not in ("dijkstra", "astar"):
        raise ValueError("planner_algorithm must be dijkstra or astar")
    hunav_profile = str(document.get("hunav_profile", "none")).lower()
    if hunav_profile not in ("none", "regular", "six"):
        raise ValueError("hunav_profile must be none, regular, or six")
    raw_seed = document.get("random_seed", 1)
    if isinstance(raw_seed, bool):
        raise ValueError("random_seed must be an integer")
    try:
        random_seed = int(raw_seed)
    except (TypeError, ValueError) as exc:
        raise ValueError("random_seed must be an integer") from exc
    if random_seed < 0 or str(raw_seed).strip() != str(random_seed):
        raise ValueError("random_seed must be a non-negative integer")
    backend = str(document.get("pedestrian_backend", "legacy_hunav"))
    if backend not in ("legacy_hunav", "multi_sfm"):
        raise ValueError("unsupported pedestrian_backend")
    config = str(document.get("pedestrian_config", ""))
    psychology = str(document.get("psychology_model", "noop"))
    if psychology != "noop":
        raise ValueError("only noop psychology is registered in v1")
    max_linear = _finite_number(document.get("max_linear", 1.0), "max_linear")
    max_angular = _finite_number(document.get("max_angular", 1.2), "max_angular")
    physics_dt = _finite_number(document.get("physics_dt", 1.0 / 60.0), "physics_dt")
    if physics_dt not in (1.0 / 60.0, 1.0 / 30.0, 1.0 / 20.0):
        raise ValueError("physics_dt must be 1/60, 1/30 or 1/20 seconds")
    if not 0 < max_linear <= 1.0 or not 0 < max_angular <= 1.2:
        raise ValueError("speed limits exceed platform bounds")
    if backend == "multi_sfm":
        if hunav_profile != "none" or not config:
            raise ValueError("multi_sfm requires pedestrian_config and hunav_profile: none")
        config = str((Path(path).resolve().parent / config).resolve())
        if not Path(config).is_file():
            raise ValueError("pedestrian_config does not exist")
        if max_linear > 0.5 or max_angular > 1.0:
            raise ValueError("multi_sfm requires explicit limits <= 0.5 m/s and 1.0 rad/s")
    elif config:
        raise ValueError("pedestrian_config is only supported by multi_sfm")
    return Scenario(
        robots=tuple(robots),
        planner_algorithm=algorithm,
        hunav_profile=hunav_profile,
        random_seed=random_seed,
        pedestrian_backend=backend,
        pedestrian_config=config,
        psychology_model=psychology,
        max_linear=max_linear,
        max_angular=max_angular,
        physics_dt=physics_dt,
    )
