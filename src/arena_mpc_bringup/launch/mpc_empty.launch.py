from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mpc_root = FindPackageShare("arena_mpc_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map_yaml")
    urdf_path = LaunchConfiguration("urdf_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("livestream", default_value="true"),
            DeclareLaunchArgument("mpc_visualization", default_value="true"),
            DeclareLaunchArgument(
                "webrtc_ip",
                default_value=EnvironmentVariable("ARENA_WEBRTC_IP", default_value="127.0.0.1"),
            ),
            DeclareLaunchArgument(
                "webrtc_signal_port",
                default_value=EnvironmentVariable("ARENA_WEBRTC_SIGNAL_PORT", default_value="49100"),
            ),
            DeclareLaunchArgument(
                "webrtc_media_port",
                default_value=EnvironmentVariable("ARENA_WEBRTC_MEDIA_PORT", default_value="47998"),
            ),
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument(
                "foxglove_address",
                default_value=EnvironmentVariable("ARENA_FOXGLOVE_ADDRESS", default_value="127.0.0.1"),
            ),
            DeclareLaunchArgument(
                "foxglove_port",
                default_value=EnvironmentVariable("ARENA_FOXGLOVE_PORT", default_value="8765"),
            ),
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
            DeclareLaunchArgument(
                "urdf_path",
                default_value=PathJoinSubstitution(
                    [EnvironmentVariable("ARENA_WS"), "config", "generated", "jackal.urdf"]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("arena_isaac"), "launch", "run_isaacsim.launch.py"]
                    )
                ),
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
                package="arena_humble_compat",
                executable="scene_bridge",
                name="arena_scene_bridge",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "urdf_path": urdf_path,
                        "dynamic_people": False,
                    }
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                arguments=[urdf_path],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="arena_mpc_bringup",
                executable="empty_human_states.py",
                name="empty_human_states",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                remappings=[("map", "/task_generator_node/map")],
                parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_yaml}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": True},
                    {"node_names": ["map_server"]},
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([mpc_root, "launch", "mpc_nav2.launch.py"])
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "mpc_visualization": LaunchConfiguration("mpc_visualization"),
                }.items(),
            ),
            Node(
                package="arena_mpc_controller",
                executable="mpc_command_watchdog",
                name="mpc_command_watchdog",
                output="screen",
                parameters=[
                    PathJoinSubstitution([mpc_root, "config", "nav2_overrides.yaml"]),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                condition=IfCondition(LaunchConfiguration("foxglove")),
                parameters=[
                    {
                        "address": LaunchConfiguration("foxglove_address"),
                        "port": ParameterValue(
                            LaunchConfiguration("foxglove_port"), value_type=int
                        ),
                        "use_sim_time": use_sim_time,
                        "capabilities": ["clientPublish", "connectionGraph", "assets"],
                    }
                ],
            ),
        ]
    )
