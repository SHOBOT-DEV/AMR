from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="shobot_twist_mux",
            executable="shobot_twist_mux_node",
            name="shobot_twist_mux",
            output="screen",
        )
    ])
