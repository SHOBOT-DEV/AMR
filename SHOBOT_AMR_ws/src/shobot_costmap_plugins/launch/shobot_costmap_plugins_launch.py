from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="shobot_costmap_plugins",
            executable="shobot_costmap_plugins_node",
            name="shobot_costmap_plugins",
            output="screen",
        )
    ])
