from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="shobot_rosbridge_suite",
            executable="shobot_rosbridge_suite_node",
            name="shobot_rosbridge_suite",
            output="screen",
        )
    ])
