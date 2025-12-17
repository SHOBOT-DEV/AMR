from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="shobot_api_bridge",
                executable="mission_api_bridge",
                name="mission_api_bridge",
                output="screen",
                parameters=[
                    {"mission_queue_topic": "/mission_queue"},
                    {"poll_interval": 2.0},
                    # API URL/token can be overridden via params or env (API_URL, API_TOKEN)
                    {"api_url": ""},
                    {"api_token": ""},
                ],
            )
        ]
    )
