from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="shobot_docking",
            executable="shobot_docking_node",
            name="shobot_docking",
            output="screen",
        )
    ])
