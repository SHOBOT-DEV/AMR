from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="shobot_robot_pose_publisher",
            executable="shobot_robot_pose_publisher_node",
            name="shobot_robot_pose_publisher",
            output="screen",
        )
    ])
