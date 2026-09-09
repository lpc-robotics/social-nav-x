"""Independent fixed-pair demo using the existing Isaac/bridge/HuNav chain."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration as LC, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


PREFIX = "/formal_social_behavior/multi"


def share(package, *parts):
    return PathJoinSubstitution([FindPackageShare(package), *parts])


def generate_launch_description():
    defaults = {
        "agent_config": share("formal_social_behavior", "config", "formal_social_multi_agents.yaml"),
        "automata_config": share("formal_social_behavior", "config", "formal_social_multi_automata.yaml"),
        "urdf_path": PathJoinSubstitution([EnvironmentVariable("ARENA_WS"), "config", "generated", "jackal.urdf"]),
        "enabled": "true", "shared_events_enabled": "true", "navigation": "false",
        "robot_x": "3.0", "robot_y": "3.0", "headless": "true", "livestream": "true", "foxglove": "true",
        "map_yaml": share("arena_simulation_setup", "worlds", "map_empty", "map", "map.yaml"),
        "webrtc_ip": EnvironmentVariable("ARENA_WEBRTC_IP", default_value="127.0.0.1"),
        "webrtc_signal_port": EnvironmentVariable("ARENA_WEBRTC_SIGNAL_PORT", default_value="49100"),
        "webrtc_media_port": EnvironmentVariable("ARENA_WEBRTC_MEDIA_PORT", default_value="47998"),
        "foxglove_address": EnvironmentVariable("ARENA_FOXGLOVE_ADDRESS", default_value="127.0.0.1"),
        "foxglove_port": EnvironmentVariable("ARENA_FOXGLOVE_PORT", default_value="8765"),
    }
    for key in ("trace_file", "steps_file", "backend_trace_file"):
        defaults[key] = EnvironmentVariable("FORMAL_MULTI_" + key.upper(), default_value="")
    actions = [DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()]
    actions += [SetEnvironmentVariable("CUDA_VISIBLE_DEVICES", EnvironmentVariable("GPU_ID", default_value="3")),
                SetEnvironmentVariable("ARENA_RENDER_GPU", EnvironmentVariable("GPU_ID", default_value="3")),
                SetEnvironmentVariable("ARENA_INTERNAL_GPU", "0")]
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
        share("arena_isaac", "launch", "run_isaacsim.launch.py")), launch_arguments={
            key: LC(key) for key in ("headless", "livestream", "webrtc_ip", "webrtc_signal_port", "webrtc_media_port")}.items()))
    actions += [
        Node(package="hunav_agent_manager", executable="hunav_loader", name="hunav_loader",
             output="screen", parameters=[LC("agent_config"), {"use_sim_time": True}]),
        Node(package="hunav_agent_manager", executable="hunav_agent_manager", name="hunav_agent_manager",
             output="screen", remappings=[("compute_agents", PREFIX + "/compute_agents_raw"),
                                            ("reset_agents", PREFIX + "/reset_agents_raw")],
             parameters=[{"use_sim_time": True, "publish_tf": True, "publish_sfm_forces": True,
                          "hunav_loader.publish_people": True}]),
        Node(package="formal_social_behavior", executable="formal_social_multi_proxy", name="formal_social_multi_proxy",
             output="screen", parameters=[{"use_sim_time": True,
                 "enabled": ParameterValue(LC("enabled"), value_type=bool),
                 "shared_events_enabled": ParameterValue(LC("shared_events_enabled"), value_type=bool),
                 "config_file": LC("automata_config"),
                 **{key: ParameterValue(LC(key), value_type=str) for key in ("trace_file", "steps_file", "backend_trace_file")}}]),
        Node(package="robot_state_publisher", executable="robot_state_publisher", name="robot_state_publisher",
             output="screen", arguments=[LC("urdf_path")], parameters=[{"use_sim_time": True}]),
        Node(package="arena_humble_compat", executable="hunav_six_behaviors_bridge", name="arena_hunav_isaac_bridge",
             output="screen", parameters=[LC("agent_config"), {"use_sim_time": True,
                 "agent_config_file": LC("agent_config"), "urdf_path": LC("urdf_path"),
                 "robot_x": ParameterValue(LC("robot_x"), value_type=float),
                 "robot_y": ParameterValue(LC("robot_y"), value_type=float),
                 "strict_six_behavior_demo": False, "status_topic": PREFIX + "/bridge_status",
                 "agent_debug_topic": PREFIX + "/hunav_agents", "ready_marker": "FORMAL_SOCIAL_MULTI_BRIDGE_READY"}]),
        Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
             condition=IfCondition(LC("navigation")), remappings=[("map", "/task_generator_node/map")],
             parameters=[{"use_sim_time": True, "yaml_filename": LC("map_yaml")}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_map",
             output="screen", condition=IfCondition(LC("navigation")), parameters=[{"use_sim_time": True,
                 "autostart": True, "node_names": ["map_server"]}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(share("arena_simulation_setup", "launch", "nav2.launch.py")),
             condition=IfCondition(LC("navigation")), launch_arguments={"robot": "jackal",
                 "task_generator_node": "/task_generator_node", "use_sim_time": "true", "global_planner": "navfn",
                 "local_planner": "dwb", "inter_planner": "navigate_w_replanning_time", "amcl": "false"}.items()),
        Node(package="foxglove_bridge", executable="foxglove_bridge", name="foxglove_bridge", output="screen",
             condition=IfCondition(LC("foxglove")), parameters=[{"use_sim_time": True,
                 "address": LC("foxglove_address"), "port": ParameterValue(LC("foxglove_port"), value_type=int),
                 "capabilities": ["clientPublish", "connectionGraph", "assets"]}]),
    ]
    return LaunchDescription(actions)
