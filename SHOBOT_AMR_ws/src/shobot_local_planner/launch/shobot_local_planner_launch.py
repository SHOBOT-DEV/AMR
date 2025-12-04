from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="shobot_local_planner",
            executable="shobot_local_planner_node",
            name="shobot_local_planner",
            output="screen",
        )
    ])
