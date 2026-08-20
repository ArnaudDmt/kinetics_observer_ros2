from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_bag = LaunchConfiguration("input_bag")
    output_bag = LaunchConfiguration("output_bag")
    play_rate = LaunchConfiguration("play_rate")
    startup_delay = LaunchConfiguration("startup_delay")

    estimator = Node(
        package="kinetics_observer_ros2",
        executable="kinetics_observer_node",
        name="kinetics_observer",
        output="screen",
    )
    recorder = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-o", output_bag, "/kinetics_observer/estimated_state"],
        output="screen",
    )
    player = ExecuteProcess(
        cmd=["ros2", "bag", "play", "--rate", play_rate, input_bag], output="screen")

    return LaunchDescription([
        DeclareLaunchArgument("input_bag"),
        DeclareLaunchArgument("output_bag", default_value="/tmp/kinetics_estimated_state"),
        DeclareLaunchArgument("play_rate", default_value="0.25"),
        DeclareLaunchArgument("startup_delay", default_value="5.0"),
        estimator,
        recorder,
        TimerAction(period=startup_delay, actions=[player]),
        RegisterEventHandler(OnProcessExit(
            target_action=player,
            on_exit=[EmitEvent(event=Shutdown(reason="Replay finished"))],
        )),
    ])
