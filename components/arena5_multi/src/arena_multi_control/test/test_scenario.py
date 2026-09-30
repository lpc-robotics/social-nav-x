from pathlib import Path

import pytest

from arena_multi_control.scenario import load_scenario


def write(tmp_path: Path, text: str) -> Path:
    path = tmp_path / "scenario.yaml"
    path.write_text(text)
    return path


def test_valid(tmp_path):
    result = load_scenario(write(tmp_path, "robots:\n- {name: robot_1, x: 1, y: 2}\nplanner_algorithm: astar\n"))
    assert result.robots[0].name == "robot_1"
    assert result.planner_algorithm == "astar"
    assert result.random_seed == 1


def test_hunav_profile(tmp_path):
    result = load_scenario(write(
        tmp_path,
        "robots:\n- {name: robot_1, x: 1, y: 2}\nhunav_profile: six\n",
    ))
    assert result.hunav_profile == "six"


@pytest.mark.parametrize("text", [
    "robots: []\n",
    "robots:\n- {name: bad, x: 1, y: 2}\n",
    "robots:\n- {name: robot_1, x: 1, y: 2}\n- {name: robot_1, x: 3, y: 4}\n",
    "robots:\n- {name: robot_1, x: .nan, y: 2}\n",
    "robots:\n- {name: robot_1, x: 1, y: 2, control_mode: broken}\n",
    "robots:\n- {name: robot_1, x: 1, y: 2}\nhunav_profile: broken\n",
    "robots:\n- {name: robot_1, x: 1, y: 2}\nrandom_seed: -1\n",
    "robots:\n- {name: robot_1, x: 1, y: 2}\nrandom_seed: 1.5\n",
])
def test_invalid(tmp_path, text):
    with pytest.raises(ValueError):
        load_scenario(write(tmp_path, text))
