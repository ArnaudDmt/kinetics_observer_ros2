from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="kinetics_observer_ros2",
            executable="kinetics_observer_node",
            name="kinetics_observer",
            output="screen",
        )
    ])
