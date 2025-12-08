from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="shobot_zone_management",
            executable="shobot_zone_management_node",
            name="shobot_zone_management",
            output="screen",
        )
    ])
