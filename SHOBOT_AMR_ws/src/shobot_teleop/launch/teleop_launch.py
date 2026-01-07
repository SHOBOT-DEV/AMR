from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_teleop = LaunchConfiguration("enable_teleop")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_teleop",
                default_value="true",
                description="Enable the keyboard teleop node (requires a TTY).",
            ),
            Node(
                package="shobot_teleop",
                executable="teleop_key",
                name="shobot_teleop",
                output="screen",
                condition=IfCondition(enable_teleop),
                parameters=[
                    {"model": "burger"},
                    {"teleop_topic": "/joy/cmd_vel"},
                ],
            )
        ]
    )
