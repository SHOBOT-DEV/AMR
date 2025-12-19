from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def include_launch(pkg, file):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory(pkg) + f"/launch/{file}"
        )
    )


def generate_launch_description():
    return LaunchDescription([
        # Ensure pose publisher is running (provides /robot_pose)
        include_launch("shobot_robot_pose_publisher", "shobot_robot_pose_publisher_launch.py"),
        Node(
            package="shobot_zone_management",
            executable="shobot_zone_management_node",
            name="shobot_zone_management",
            output="screen",
        )
    ])
