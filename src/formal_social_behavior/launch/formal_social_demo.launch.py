from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    agent_config = LaunchConfiguration("agent_config")
    automata_config = LaunchConfiguration("automata_config")
    trace_file = LaunchConfiguration("trace_file")
    urdf_path = LaunchConfiguration("urdf_path")
    headless = LaunchConfiguration("headless")
    livestream = LaunchConfiguration("livestream")
    webrtc_ip = LaunchConfiguration("webrtc_ip")
    webrtc_signal_port = LaunchConfiguration("webrtc_signal_port")
    webrtc_media_port = LaunchConfiguration("webrtc_media_port")
    foxglove = LaunchConfiguration("foxglove")
    foxglove_address = LaunchConfiguration("foxglove_address")
    foxglove_port = LaunchConfiguration("foxglove_port")
    navigation = LaunchConfiguration("navigation")
    map_yaml = LaunchConfiguration("map_yaml")

    isaac_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("arena_isaac"), "launch", "run_isaacsim.launch.py"]
            )
        ),
        launch_arguments={
            "headless": headless,
            "livestream": livestream,
            "webrtc_ip": webrtc_ip,
            "webrtc_signal_port": webrtc_signal_port,
            "webrtc_media_port": webrtc_media_port,
            "log_level": "info",
        }.items(),
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        condition=IfCondition(navigation),
        remappings=[("map", "/task_generator_node/map")],
        parameters=[{"use_sim_time": True, "yaml_filename": map_yaml}],
    )
    map_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        condition=IfCondition(navigation),
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["map_server"],
            }
        ],
    )
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("arena_simulation_setup"),
                    "launch",
                    "nav2.launch.py",
                ]
            )
        ),
        condition=IfCondition(navigation),
        launch_arguments={
            "robot": "jackal",
            "task_generator_node": "/task_generator_node",
            "use_sim_time": "true",
            "global_planner": "navfn",
            "local_planner": "dwb",
            "inter_planner": "navigate_w_replanning_time",
            "amcl": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "agent_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("formal_social_behavior"),
                        "config",
                        "formal_social_agent.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "automata_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("formal_social_behavior"),
                        "config",
                        "formal_social_automata.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "trace_file",
                default_value=EnvironmentVariable(
                    "FORMAL_SOCIAL_TRACE_FILE", default_value=""
                ),
            ),
            DeclareLaunchArgument(
                "urdf_path",
                default_value=PathJoinSubstitution(
                    [
                        EnvironmentVariable("ARENA_WS"),
                        "config",
                        "generated",
                        "jackal.urdf",
                    ]
                ),
            ),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("livestream", default_value="true"),
            DeclareLaunchArgument(
                "webrtc_ip",
                default_value=EnvironmentVariable(
                    "ARENA_WEBRTC_IP", default_value="127.0.0.1"
                ),
            ),
            DeclareLaunchArgument(
                "webrtc_signal_port",
                default_value=EnvironmentVariable(
                    "ARENA_WEBRTC_SIGNAL_PORT", default_value="49100"
                ),
            ),
            DeclareLaunchArgument(
                "webrtc_media_port",
                default_value=EnvironmentVariable(
                    "ARENA_WEBRTC_MEDIA_PORT", default_value="47998"
                ),
            ),
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument(
                "foxglove_address",
                default_value=EnvironmentVariable(
                    "ARENA_FOXGLOVE_ADDRESS", default_value="127.0.0.1"
                ),
            ),
            DeclareLaunchArgument(
                "foxglove_port",
                default_value=EnvironmentVariable(
                    "ARENA_FOXGLOVE_PORT", default_value="8765"
                ),
            ),
            DeclareLaunchArgument("navigation", default_value="false"),
            DeclareLaunchArgument(
                "map_yaml",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("arena_simulation_setup"),
                        "worlds",
                        "map_empty",
                        "map",
                        "map.yaml",
                    ]
                ),
            ),
            SetEnvironmentVariable(
                "CUDA_VISIBLE_DEVICES",
                EnvironmentVariable("GPU_ID", default_value="3"),
            ),
            SetEnvironmentVariable(
                "ARENA_RENDER_GPU",
                EnvironmentVariable("GPU_ID", default_value="3"),
            ),
            SetEnvironmentVariable("ARENA_INTERNAL_GPU", "0"),
            isaac_launch,
            Node(
                package="hunav_agent_manager",
                executable="hunav_loader",
                name="hunav_loader",
                output="screen",
                parameters=[agent_config, {"use_sim_time": True}],
            ),
            Node(
                package="hunav_agent_manager",
                executable="hunav_agent_manager",
                name="hunav_agent_manager",
                output="screen",
                remappings=[
                    (
                        "compute_agents",
                        "/formal_social_behavior/compute_agents_raw",
                    ),
                    (
                        "reset_agents",
                        "/formal_social_behavior/reset_agents_raw",
                    ),
                ],
                parameters=[
                    {
                        "use_sim_time": True,
                        "publish_tf": True,
                        "publish_sfm_forces": True,
                        "hunav_loader.publish_people": True,
                    }
                ],
            ),
            Node(
                package="formal_social_behavior",
                executable="formal_social_behavior_proxy",
                name="formal_social_behavior_proxy",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "enabled": True,
                        "config_file": automata_config,
                        "trace_file": ParameterValue(trace_file, value_type=str),
                    }
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                arguments=[urdf_path],
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="arena_humble_compat",
                executable="hunav_six_behaviors_bridge",
                name="arena_hunav_isaac_bridge",
                output="screen",
                parameters=[
                    agent_config,
                    {
                        "agent_config_file": agent_config,
                        "urdf_path": urdf_path,
                        "use_sim_time": True,
                        "strict_six_behavior_demo": False,
                        "status_topic": "/formal_social_behavior/bridge_status",
                        "agent_debug_topic": (
                            "/formal_social_behavior/hunav_agents"
                        ),
                        "ready_marker": "FORMAL_SOCIAL_BRIDGE_READY",
                    },
                ],
            ),
            map_server,
            map_lifecycle_manager,
            nav2_launch,
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                condition=IfCondition(foxglove),
                parameters=[
                    {
                        "address": foxglove_address,
                        "port": ParameterValue(foxglove_port, value_type=int),
                        "use_sim_time": True,
                        "capabilities": [
                            "clientPublish",
                            "connectionGraph",
                            "assets",
                        ],
                    }
                ],
            ),
        ]
    )
