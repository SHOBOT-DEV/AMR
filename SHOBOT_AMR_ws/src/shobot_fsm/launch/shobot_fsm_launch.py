from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="shobot_fsm",
                executable="shobot_fsm_node",
                name="shobot_fsm",
                output="screen",
            )
        ]
    )
