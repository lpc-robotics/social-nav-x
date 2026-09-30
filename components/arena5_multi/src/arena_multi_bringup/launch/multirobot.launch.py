from __future__ import annotations

import copy
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from arena_multi_control.scenario import load_scenario
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _write_robot_params(base_path: Path, robot, peers, algorithm: str, max_linear=1.0, max_angular=1.2,
                        pedestrian_backend="legacy_hunav") -> str:
    data = yaml.safe_load(base_path.read_text(encoding="utf-8"))
    base_frame = f"{robot.name}/base_link"
    odom_frame = f"{robot.name}/odom"
    data["controller_server"]["ros__parameters"]["use_sim_time"] = True
    local = data["local_costmap"]["local_costmap"]["ros__parameters"]
    local["global_frame"] = odom_frame
    local["robot_base_frame"] = base_frame
    if peers:
        local["peer_layer"]["peers"] = peers
    else:
        local["peer_layer"].pop("peers", None)
        local["peer_layer"]["enabled"] = False
    local["voxel_layer"]["normalized"]["topic"] = f"/{robot.name}/lidar_normalized"
    if pedestrian_backend == "multi_sfm":
        # A single planar scan should clear planar cells regardless of small
        # sensor-height changes across voxel boundaries. Keep legacy parameters
        # unchanged and use Nav2's planar layer in the new low-speed profile.
        observation = local["voxel_layer"]["normalized"]
        local["voxel_layer"] = {
            "plugin": "nav2_costmap_2d::ObstacleLayer", "enabled": True,
            "max_obstacle_height": 2.0, "observation_sources": "normalized",
            "normalized": observation,
        }
    global_map = data["global_costmap"]["global_costmap"]["ros__parameters"]
    global_map["robot_base_frame"] = base_frame
    data["planner_server"]["ros__parameters"]["GridBased"]["use_astar"] = algorithm == "astar"
    navigator = data["bt_navigator"]["ros__parameters"]
    navigator["robot_base_frame"] = base_frame
    navigator["odom_topic"] = f"/{robot.name}/odom"
    navigator["default_nav_to_pose_bt_xml"] = str(
        base_path.parent.parent / "behavior_trees"
        / "navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml"
    )
    behavior = data["behavior_server"]["ros__parameters"]
    behavior["global_frame"] = odom_frame
    behavior["robot_base_frame"] = base_frame
    data["velocity_smoother"]["ros__parameters"]["odom_topic"] = f"/{robot.name}/odom"
    controller = data["controller_server"]["ros__parameters"]["FollowPath"]
    controller.update(max_vel_x=max_linear, max_speed_xy=max_linear, max_vel_theta=max_angular)
    smoother = data["velocity_smoother"]["ros__parameters"]
    smoother["max_velocity"] = [max_linear, 0.0, max_angular]
    smoother["min_velocity"] = [0.0, 0.0, -max_angular]
    behavior["max_rotational_vel"] = max_angular

    run_root = Path(os.environ.get("ARENA_MULTI_RUN_DIR", "/tmp/arena5_multi_params"))
    output_dir = run_root / "generated_params"
    output_dir.mkdir(parents=True, exist_ok=True)
    output = output_dir / f"{robot.name}.yaml"
    output.write_text(yaml.safe_dump({robot.name: data}, sort_keys=False), encoding="utf-8")
    return str(output)


def _nav2_nodes(robot, params_file: str):
    namespace = robot.name
    common = {"namespace": namespace, "output": "screen", "parameters": [params_file]}
    nodes = [
        Node(package="nav2_controller", executable="controller_server", name="controller_server",
             remappings=[("cmd_vel", "cmd_vel_raw")], **common),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", **common),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server",
             remappings=[("cmd_vel", "cmd_vel_nav")], **common),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", **common),
        Node(package="nav2_waypoint_follower", executable="waypoint_follower", name="waypoint_follower", **common),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother", name="velocity_smoother",
             remappings=[("cmd_vel", "cmd_vel_raw"), ("cmd_vel_smoothed", "cmd_vel_nav")], **common),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            namespace=namespace,
            output="screen",
            parameters=[{
                "use_sim_time": True,
                # The scenario manager starts this lifecycle group only after
                # every SpawnUrdf call (and its World.reset()) has completed.
                "autostart": False,
                "bond_timeout": 0.0,
                "node_names": [
                    "controller_server", "planner_server", "behavior_server",
                    "bt_navigator", "waypoint_follower", "velocity_smoother",
                ],
            }],
        ),
    ]
    return nodes


def _launch(context):
    scenario_path = Path(LaunchConfiguration("scenario").perform(context)).resolve()
    urdf_path = Path(LaunchConfiguration("urdf").perform(context)).resolve()
    map_path = Path(LaunchConfiguration("map").perform(context)).resolve()
    world_path = Path(LaunchConfiguration("world").perform(context)).resolve()
    scenario = load_scenario(scenario_path)
    robot_names = [robot.name for robot in scenario.robots]
    if not urdf_path.is_file() or not map_path.is_file() or not world_path.is_file():
        raise RuntimeError("URDF, map, or world file is missing")

    arena_isaac_share = get_package_share_directory("arena_isaac")
    bringup_share = Path(get_package_share_directory("arena_multi_bringup"))
    nav2_params = bringup_share / "config/nav2_multirobot.yaml"
    urdf_xml = urdf_path.read_text(encoding="utf-8")
    actions = [
        SetEnvironmentVariable("ARENA_ROBOT_NAMES", ",".join(robot_names)),
        SetEnvironmentVariable("ARENA_RANDOM_SEED", str(scenario.random_seed)),
        SetEnvironmentVariable("ARENA_NORMALIZED_SCAN_SEED", str(scenario.random_seed)),
        SetEnvironmentVariable("ARENA_IDEAL_MAX_LINEAR", str(scenario.max_linear)),
        SetEnvironmentVariable("ARENA_IDEAL_MAX_ANGULAR", str(scenario.max_angular)),
        SetEnvironmentVariable("ARENA_PHYSICS_DT", str(scenario.physics_dt)),
        SetEnvironmentVariable("ARENA_PEDESTRIAN_TELEMETRY", str(scenario.pedestrian_backend == "multi_sfm").lower()),
        SetEnvironmentVariable("ARENA_MULTI_CLOCK_ALIGNMENT", str(scenario.pedestrian_backend == "multi_sfm").lower()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(arena_isaac_share) / "launch/run_isaacsim.launch.py")),
            launch_arguments={
                "headless": LaunchConfiguration("headless"),
                "livestream": LaunchConfiguration("livestream"),
                "webrtc_ip": LaunchConfiguration("webrtc_ip"),
                "webrtc_signal_port": LaunchConfiguration("webrtc_signal_port"),
                "webrtc_media_port": LaunchConfiguration("webrtc_media_port"),
                "log_level": "info",
            }.items(),
        ),
        Node(
            package="nav2_map_server", executable="map_server", name="map_server", output="screen",
            parameters=[{"use_sim_time": True, "yaml_filename": str(map_path), "topic_name": "/map", "frame_id": "map"}],
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_map",
            output="screen", parameters=[{"use_sim_time": True, "autostart": True, "node_names": ["map_server"]}],
        ),
        Node(
            package="arena_multi_bringup", executable="scenario_manager", name="scenario_manager",
            output="screen", parameters=[{
                "use_sim_time": True, "scenario_file": str(scenario_path),
                "urdf_path": str(urdf_path), "world_file": str(world_path),
            }],
        ),
    ]

    for robot in scenario.robots:
        peers = [name for name in robot_names if name != robot.name]
        guard_parameters = {
            "robot_name": robot.name,
            "control_mode": robot.control_mode,
            "max_linear": scenario.max_linear,
            "max_angular": scenario.max_angular,
            "require_pedestrian_health": scenario.pedestrian_backend == "multi_sfm",
        }
        if peers:
            guard_parameters["peer_names"] = peers
        actions.extend([
            Node(
                package="robot_state_publisher", executable="robot_state_publisher",
                namespace=robot.name, name="robot_state_publisher", output="screen",
                parameters=[{"use_sim_time": True, "robot_description": urdf_xml, "frame_prefix": robot.name + "/"}],
                remappings=[("tf", "/tf"), ("tf_static", "/tf_static"), ("joint_states", "joint_states")],
            ),
            Node(
                package="arena_multi_control", executable="command_guard", namespace=robot.name,
                name="command_guard", output="screen",
                parameters=[guard_parameters],
            ),
            Node(
                package="arena_multi_control", executable="wheel_visualizer", namespace=robot.name,
                name="wheel_visualizer", output="screen", parameters=[{"robot_name": robot.name}],
            ),
        ])
        if robot.control_mode == "nav2":
            params_file = _write_robot_params(nav2_params, robot, peers, scenario.planner_algorithm,
                                             scenario.max_linear, scenario.max_angular, scenario.pedestrian_backend)
            actions.extend(_nav2_nodes(robot, params_file))

    if scenario.pedestrian_backend == "multi_sfm":
        world = yaml.safe_load(world_path.read_text())
        actions.extend([
            Node(package="arena_multi_hunav_core", executable="multi_sfm_server", output="screen",
                 parameters=[{"use_sim_time": True, "world_width": float(world["width"]),
                              "world_height": float(world["height"])}]),
            Node(package="arena_multi_hunav", executable="multi_sfm_adapter", output="screen",
                 parameters=[{"use_sim_time": True, "robot_names": robot_names,
                              "agent_config_file": scenario.pedestrian_config,
                              "random_seed": scenario.random_seed,
                              "psychology_model": scenario.psychology_model}]),
        ])
    elif scenario.hunav_profile != "none":
        root = Path(os.environ.get("ARENA_MULTI_WS", "/home/lpc/workspace/arena5_multi_ws"))
        hunav_config = root / "config/hunav" / (
            "regular.yaml" if scenario.hunav_profile == "regular" else "six_behaviors.yaml"
        )
        if not hunav_config.is_file():
            raise RuntimeError(f"HuNav configuration is missing: {hunav_config}")
        actions.extend([
            Node(
                package="hunav_agent_manager", executable="hunav_loader", name="hunav_loader",
                output="screen", parameters=[str(hunav_config), {"use_sim_time": True}],
            ),
            Node(
                package="hunav_agent_manager", executable="hunav_agent_manager", name="hunav_agent_manager",
                output="screen", parameters=[{
                    "use_sim_time": True,
                    "publish_tf": True,
                    "publish_sfm_forces": True,
                    "hunav_loader.publish_people": True,
                }],
            ),
            Node(
                package="arena_multi_hunav", executable="hunav_multi_adapter",
                name="arena_hunav_isaac_bridge", output="screen",
                parameters=[str(hunav_config), {
                    "agent_config_file": str(hunav_config),
                    "reference_robot": "robot_1",
                    "strict_six_behavior_demo": scenario.hunav_profile == "six",
                    "status_topic": "/multirobot/hunav/status",
                    "agent_debug_topic": "/multirobot/hunav/agents",
                    "ready_marker": "MULTIROBOT_HUNAV_COMPUTE_READY",
                    "use_sim_time": True,
                }],
            ),
        ])

    if LaunchConfiguration("foxglove").perform(context).lower() == "true":
        actions.append(Node(
            package="foxglove_bridge", executable="foxglove_bridge", name="foxglove_bridge", output="screen",
            parameters=[{
                "address": LaunchConfiguration("foxglove_address"),
                "port": ParameterValue(LaunchConfiguration("foxglove_port"), value_type=int),
                "use_sim_time": True,
                "capabilities": ["clientPublish", "connectionGraph", "assets"],
            }],
        ))
    return actions


def generate_launch_description():
    root = Path(os.environ.get("ARENA_MULTI_WS", "/home/lpc/workspace/arena5_multi_ws"))
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value=str(root / "config/scenarios/two_robots_dijkstra.yaml")),
        DeclareLaunchArgument("urdf", default_value=str(root / "config/urdf/jackal.urdf")),
        DeclareLaunchArgument("map", default_value=str(root / "config/maps/map.yaml")),
        DeclareLaunchArgument("world", default_value=str(root / "config/world/arena.yaml")),
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("livestream", default_value="true"),
        DeclareLaunchArgument("webrtc_ip", default_value=os.environ.get("ARENA_WEBRTC_IP", "127.0.0.1")),
        DeclareLaunchArgument("webrtc_signal_port", default_value=os.environ.get("ARENA_WEBRTC_SIGNAL_PORT", "49130")),
        DeclareLaunchArgument("webrtc_media_port", default_value=os.environ.get("ARENA_WEBRTC_MEDIA_PORT", "48030")),
        DeclareLaunchArgument("foxglove", default_value="true"),
        DeclareLaunchArgument("foxglove_address", default_value=os.environ.get("ARENA_FOXGLOVE_ADDRESS", "127.0.0.1")),
        DeclareLaunchArgument("foxglove_port", default_value=os.environ.get("ARENA_FOXGLOVE_PORT", "8795")),
        OpaqueFunction(function=_launch),
    ])
