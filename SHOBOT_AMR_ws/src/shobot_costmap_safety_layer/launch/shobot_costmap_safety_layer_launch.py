from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="shobot_costmap_safety_layer",
            executable="shobot_costmap_safety_layer_node",
            name="shobot_costmap_safety_layer",
            output="screen",
        )
    ])
