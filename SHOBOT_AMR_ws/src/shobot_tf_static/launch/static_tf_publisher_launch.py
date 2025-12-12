from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration("config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("shobot_tf_static"), "config", "static_transforms.yaml"]
                ),
                description="Path to static transform parameters.",
            ),
            Node(
                package="shobot_tf_static",
                executable="static_tf_publisher_node",
                name="shobot_static_tf_publisher",
                output="screen",
                parameters=[config],
            ),
        ]
    )
