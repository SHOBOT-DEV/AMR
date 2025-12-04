from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="shobot_status_aggregator",
            executable="shobot_status_aggregator_node",
            name="shobot_status_aggregator",
            output="screen",
        )
    ])
