from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="shobot_api_bridge",
                executable="api_bridge",
                name="shobot_api_bridge",
                output="screen",
                parameters=[
                    {"cmd_vel_topic": "/cmd_vel"},
                    {"status_topic": "/status_text"},
                ],
            )
        ]
    )
