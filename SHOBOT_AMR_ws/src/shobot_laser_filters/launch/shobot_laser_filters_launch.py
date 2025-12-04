from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="shobot_laser_filters",
            executable="shobot_laser_filters_node",
            name="shobot_laser_filters",
            output="screen",
        )
    ])
