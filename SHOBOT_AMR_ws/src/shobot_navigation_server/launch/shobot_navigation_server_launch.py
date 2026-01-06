from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="shobot_navigation_server",
            executable="shobot_navigation_server_node",
            name="shobot_navigation_server",
            output="screen",
            parameters=[
                {"nav2_action_name": "/navigate_to_pose"},
                {"status_topic": "/navigation_server/status"},
            ],
        )
    ])
