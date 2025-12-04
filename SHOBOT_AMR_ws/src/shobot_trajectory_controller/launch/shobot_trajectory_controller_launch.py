from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="shobot_trajectory_controller",
            executable="shobot_trajectory_controller_node",
            name="shobot_trajectory_controller",
            output="screen",
        )
    ])
