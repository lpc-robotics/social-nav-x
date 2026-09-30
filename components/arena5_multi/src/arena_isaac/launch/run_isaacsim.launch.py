import launch
from arena_bringup.substitutions import LaunchArgument
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import ExecutableInPackage


def generate_launch_description():
    ld = []
    LaunchArgument.auto_append(ld)

    logger = LaunchArgument(
        name='log_level',
        default_value='debug',
        description='Logging level',
    )

    headless = LaunchArgument(
        name='headless',
        default_value='False',
    )

    livestream = LaunchArgument(
        name='livestream',
        default_value='False',
    )

    webrtc_ip = LaunchArgument(
        name='webrtc_ip',
        default_value=EnvironmentVariable('ARENA_WEBRTC_IP', default_value='127.0.0.1'),
    )

    webrtc_signal_port = LaunchArgument(
        name='webrtc_signal_port',
        default_value='49100',
    )

    webrtc_media_port = LaunchArgument(
        name='webrtc_media_port',
        default_value='47998',
    )

    run_isaacsim_path = ExecutableInPackage(
        executable='run_isaacsim',
        package='arena_isaac',
    )

    return LaunchDescription([
        *ld,
        launch.actions.DeclareLaunchArgument(
            "log_level",
            default_value=["debug"],
            description="Logging level",
        ),
        ExecuteProcess(
            cmd=[
                EnvironmentVariable(
                    'ISAAC_PYTHON',
                    default_value=PathJoinSubstitution([EnvironmentVariable('ISAAC_PATH'), 'python.sh']),
                ),
                run_isaacsim_path,
                '--log-level', logger.substitution,
                '--headless', headless.substitution,
                '--livestream', livestream.substitution,
                '--webrtc-ip', webrtc_ip.substitution,
                '--webrtc-signal-port', webrtc_signal_port.substitution,
                '--webrtc-media-port', webrtc_media_port.substitution,
            ],
            output='screen',
        ),
    ])
