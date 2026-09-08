from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mpc_root = FindPackageShare("arena_mpc_bringup")
    map_yaml = LaunchConfiguration("map_yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("livestream", default_value="true"),
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("arena_bringup"), "launch", "isaac_six_behaviors.launch.py"]
                    )
                ),
                launch_arguments={
                    "headless": LaunchConfiguration("headless"),
                    "livestream": LaunchConfiguration("livestream"),
                    "webrtc_ip": LaunchConfiguration("webrtc_ip"),
                    "webrtc_signal_port": LaunchConfiguration("webrtc_signal_port"),
                    "webrtc_media_port": LaunchConfiguration("webrtc_media_port"),
                    "foxglove": LaunchConfiguration("foxglove"),
                    "foxglove_address": LaunchConfiguration("foxglove_address"),
                    "foxglove_port": LaunchConfiguration("foxglove_port"),
                    "navigation": "false",
                    "map_yaml": map_yaml,
                }.items(),
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
                launch_arguments={"use_sim_time": use_sim_time}.items(),
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
        ]
    )
