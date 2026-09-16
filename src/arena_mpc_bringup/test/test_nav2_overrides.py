import os
from pathlib import Path

import yaml


def _parameters(config, node_name):
    return config[node_name][node_name]["ros__parameters"]


def test_costmap_layer_contract():
    config_path = Path(os.environ["MPC_NAV2_OVERRIDES_PATH"])
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))

    global_params = _parameters(config, "global_costmap")
    assert global_params["plugins"] == ["static_layer", "inflation_layer"]
    assert "obstacle_layer" not in global_params

    local_params = _parameters(config, "local_costmap")
    voxel = local_params["voxel_layer"]
    assert voxel["observation_sources"] == "lidar depth_clearing"
    assert voxel["lidar"]["expected_update_rate"] == 0.3

    depth = voxel["depth_clearing"]
    assert depth["topic"] == "/lidar_clearing"
    assert depth["data_type"] == "PointCloud2"
    assert depth["marking"] is False
    assert depth["clearing"] is True
    assert depth["raytrace_max_range"] == 3.0
    assert depth["observation_persistence"] == 0.0

    watchdog_params = config["mpc_command_watchdog"]["ros__parameters"]
    assert watchdog_params["costmap_topic"] == "/local_costmap/costmap_raw"
