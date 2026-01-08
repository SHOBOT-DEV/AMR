from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    api_host = LaunchConfiguration("api_host")
    api_port = LaunchConfiguration("api_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "api_host",
                default_value="0.0.0.0",
                description="HTTP server bind host for the API bridge.",
            ),
            DeclareLaunchArgument(
                "api_port",
                default_value="8000",
                description="HTTP server port for the API bridge.",
            ),
            Node(
                package="shobot_api_bridge",
                executable="api_bridge",
                name="shobot_api_bridge",
                output="screen",
                parameters=[
                    {"cmd_vel_topic": "/cmd_vel"},
                    {"status_topic": "/status_text"},
                    {"api_host": api_host},
                    {"api_port": api_port},
                ],
            )
        ]
    )
