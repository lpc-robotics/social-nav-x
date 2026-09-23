import math
import os
import sys
from pathlib import Path

import numpy as np


runtime_root = Path(os.environ["MPC_DEPTH_CLEARING_RUNTIME"])
sys.path.insert(0, str(runtime_root))
sys.dont_write_bytecode = True

from isaac_utils.clearing_geometry import (  # noqa: E402
    clearing_endpoints,
    horizontal_directions,
)


def test_invalid_depth_cannot_clear_and_positive_infinity_is_bounded():
    values = [-1, 0, np.nan, -np.inf, 0.03, 0.08, 2.0, np.inf, 100.0]
    rays = np.tile([1, 0, 0], (len(values), 1))
    points = clearing_endpoints(values, rays, 0.08, 12)
    np.testing.assert_allclose(points, [[1.95, 0, 0], [12, 0, 0], [12, 0, 0]])


def test_projection_preserves_radial_distance_and_camera_axes():
    for quadrant in range(4):
        yaw = quadrant * math.pi / 2
        rays = horizontal_directions(321, yaw)
        np.testing.assert_allclose(np.linalg.norm(rays, axis=1), 1)
        np.testing.assert_allclose(
            rays[160], [math.cos(yaw), math.sin(yaw), 0], atol=1e-15
        )
        points = clearing_endpoints(np.full(321, 3.0), rays, 0.08, 12)
        np.testing.assert_allclose(np.linalg.norm(points, axis=1), 2.95, rtol=1e-6)


def test_runtime_hooks_create_and_update_depth_clearing():
    lidar_text = (
        runtime_root / "isaac_utils/graphs/sensors/lidar.py"
    ).read_text(encoding="utf-8")
    runner_text = (runtime_root / "arena_isaac/run_isaacsim.py").read_text(
        encoding="utf-8"
    )

    assert 'os.environ.get("ARENA_DEPTH_CLEARING", "false")' in lidar_text
    assert "scan_topic + \"_clearing\"" in lidar_text
    assert "self._depth_clearing.destroy()" in lidar_text
    assert "DepthClearing.update_all(world.current_time)" in runner_text
