from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="shobot_pointcloud_to_laserscan",
            executable="shobot_pointcloud_to_laserscan_node",
            name="shobot_pointcloud_to_laserscan",
            output="screen",
        )
    ])
