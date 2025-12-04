from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="shobot_mission_control",
            executable="shobot_mission_control_node",
            name="shobot_mission_control",
            output="screen",
        )
    ])
