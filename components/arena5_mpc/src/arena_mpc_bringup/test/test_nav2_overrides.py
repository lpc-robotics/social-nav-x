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

    progress = config["controller_server"]["ros__parameters"]["progress_checker"]
    assert progress["plugin"] == "arena_mpc_controller::SafetyAwareProgressChecker"
    assert progress["required_movement_radius"] == 0.05
    assert progress["required_movement_angle"] == 0.1
    assert progress["movement_time_allowance"] == 120.0
    assert progress["status_topic"] == "/FollowPath/status"
    assert progress["status_timeout"] == 1.0


def test_mpc_speed_limits_match_velocity_smoother():
    model_path = Path(os.environ["MPC_CONTROLLER_MODEL_PATH"])
    model = yaml.safe_load(model_path.read_text(encoding="utf-8"))
    controller = model["controller_plugins_dict"]["FollowPath"]

    overrides_path = Path(os.environ["MPC_NAV2_OVERRIDES_PATH"])
    overrides = yaml.safe_load(overrides_path.read_text(encoding="utf-8"))
    smoother = overrides["velocity_smoother"]["ros__parameters"]

    assert controller["max_linear"] == 0.8
    assert controller["costmap_obstacle_wait_limit"] == 120.0
    assert controller["goal_position_tolerance_fallback"] == 0.25
    assert controller["max_angular"] == 1.5
    assert controller["max_initial_clearance_violation"] == 0.05
    assert controller["clearance_recovery_exit"] == 0.10
    assert controller["clearance_recovery_linear"] == 0.40
    assert controller["clearance_recovery_angular"] == 1.0
    assert controller["clearance_recovery_min_outward_cos"] == 0.10
    assert controller["solver_budget_ms"] == 75.0
    assert controller["plugin_commit_limit_ms"] == 90.0
    assert smoother["max_velocity"] == [0.8, 0.0, 1.5]
    assert smoother["min_velocity"] == [-0.8, 0.0, -1.5]


def test_dwb_speed_limits_match_velocity_smoother():
    config_path = Path(os.environ["DWB_SPEED_OVERRIDES_PATH"])
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    controller = config["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = config["velocity_smoother"]["ros__parameters"]

    assert controller["max_vel_x"] == 0.8
    assert controller["max_speed_xy"] == 0.8
    assert controller["max_vel_theta"] == 1.5
    assert smoother["max_velocity"] == [0.8, 0.0, 1.5]
    assert smoother["min_velocity"] == [-0.8, 0.0, -1.5]
