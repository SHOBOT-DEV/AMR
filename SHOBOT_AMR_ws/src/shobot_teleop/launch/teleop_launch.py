from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="shobot_teleop",
                executable="teleop_key",
                name="shobot_teleop",
                output="screen",
                parameters=[
                    {"model": "burger"},
                    {"teleop_topic": "/joy/cmd_vel"},
                ],
            )
        ]
    )
