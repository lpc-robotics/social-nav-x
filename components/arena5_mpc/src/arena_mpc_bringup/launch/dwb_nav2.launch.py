from arena_bringup.substitutions import (
    YAMLFileSubstitution,
    YAMLMergeSubstitution,
    YAMLReplaceSubstitution,
    YAMLRetrieveSubstitution,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    arena_root = FindPackageShare("arena_simulation_setup")
    bringup_root = FindPackageShare("arena_mpc_bringup")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    model_values = YAMLMergeSubstitution(
        YAMLFileSubstitution(
            PathJoinSubstitution([arena_root, "configs", "nav2", "model_params.yaml"])
        ),
        YAMLFileSubstitution(
            PathJoinSubstitution(
                [arena_root, "entities", "robots", "jackal", "model_params.yaml"]
            )
        ),
        YAMLFileSubstitution(
            PathJoinSubstitution(
                [
                    arena_root,
                    "configs",
                    "nav2",
                    "controllers",
                    "dwb",
                    "controller_config.yaml",
                ]
            )
        ),
        YAMLFileSubstitution(
            PathJoinSubstitution(
                [arena_root, "configs", "nav2", "planners", "navfn", "planner_config.yaml"]
            )
        ),
        YAMLFileSubstitution(
            PathJoinSubstitution(
                [
                    arena_root,
                    "configs",
                    "nav2",
                    "interplanners",
                    "navigate_w_replanning_time",
                    "interplanner_config.yaml",
                ]
            )
        ),
        YAMLFileSubstitution.from_dict(
            {
                "frame": "",
                "task_generator_node": "/task_generator_node",
                "namespace": "",
                "default_nav_to_pose_bt_xml": YAMLRetrieveSubstitution(
                    YAMLFileSubstitution(
                        PathJoinSubstitution(
                            [
                                arena_root,
                                "configs",
                                "nav2",
                                "interplanners",
                                "navigate_w_replanning_time",
                                "interplanner_config.yaml",
                            ]
                        )
                    ),
                    "bt_navigator/ros__parameters/default_nav_to_pose_bt_xml",
                ),
                "default_nav_through_poses_bt_xml": "",
                "plugin_lib_names": YAMLRetrieveSubstitution(
                    YAMLFileSubstitution(
                        PathJoinSubstitution(
                            [
                                arena_root,
                                "configs",
                                "nav2",
                                "interplanners",
                                "navigate_w_replanning_time",
                                "interplanner_config.yaml",
                            ]
                        )
                    ),
                    "bt_navigator/ros__parameters/plugin_lib_names",
                ),
            },
            substitute=True,
        ),
    )

    arena_parameters = YAMLReplaceSubstitution(
        obj=YAMLFileSubstitution(
            PathJoinSubstitution([arena_root, "configs", "nav2", "nav2.yaml"])
        ),
        substitutions=YAMLFileSubstitution(model_values),
    )
    merged_parameters = YAMLMergeSubstitution(
        YAMLFileSubstitution(arena_parameters),
        YAMLFileSubstitution(
            PathJoinSubstitution(
                [bringup_root, "config", "dwb_speed_overrides.yaml"]
            )
        ),
    )
    configured_parameters = ParameterFile(
        RewrittenYaml(
            source_file=merged_parameters,
            root_key=namespace,
            param_rewrites={"use_sim_time": use_sim_time, "autostart": "true"},
            convert_types=True,
        ),
        allow_substs=True,
    )

    common = {
        "output": "screen",
        "parameters": [configured_parameters],
        "arguments": ["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        "remappings": [("/tf", "tf"), ("/tf_static", "tf_static")],
    }
    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            Node(
                package="nav2_controller",
                executable="controller_server",
                additional_env={"OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1"},
                remappings=common["remappings"] + [("cmd_vel", "cmd_vel_nav")],
                **{key: value for key, value in common.items() if key != "remappings"},
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                **common,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                remappings=common["remappings"] + [("map", "/task_generator_node/map")],
                **{key: value for key, value in common.items() if key != "remappings"},
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                **common,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                **common,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                **common,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                remappings=common["remappings"]
                + [
                    ("cmd_vel", "cmd_vel_nav"),
                    ("cmd_vel_smoothed", "cmd_vel"),
                ],
                **{key: value for key, value in common.items() if key != "remappings"},
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": True},
                    {"node_names": lifecycle_nodes},
                ],
            ),
        ]
    )
