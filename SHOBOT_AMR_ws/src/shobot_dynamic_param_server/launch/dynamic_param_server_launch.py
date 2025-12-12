from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    propagate_targets = LaunchConfiguration("propagate_targets")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "propagate_targets",
                default_value="[]",
                description="List of node names to push parameter changes to.",
            ),
            Node(
                package="shobot_dynamic_param_server",
                executable="dynamic_param_server_node",
                name="shobot_dynamic_param_server",
                output="screen",
                parameters=[{"propagate_targets": propagate_targets}],
            ),
        ]
    )
