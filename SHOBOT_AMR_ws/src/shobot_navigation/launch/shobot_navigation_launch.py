from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="shobot_navigation",
            executable="shobot_navigation_node",
            name="shobot_navigation",
            output="screen",
            parameters=[
                {"goal_topic": "/goal_pose"},
                {"status_topic": "/navigation_status"},
                {"feedback_topic": "/navigation_feedback"},
                {"nav2_action_name": "navigate_to_pose"},
            ],
        )
    ])
