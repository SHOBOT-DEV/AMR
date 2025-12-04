from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="shobot_scan_merger",
            executable="shobot_scan_merger_node",
            name="shobot_scan_merger",
            output="screen",
        )
    ])
