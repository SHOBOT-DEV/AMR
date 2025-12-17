from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="shobot_mission_control",
            executable="shobot_mission_control_node",
            name="shobot_mission_control",
            output="screen",
            parameters=[
                {"mission_topic": "/mission_queue"},
                {"status_topic": "/mission_status"},
                {"nav2_action_name": "navigate_to_pose"},
                {"dock_action_name": "dock"},
            ],
        )
    ])
