from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="shobot_docking",
            executable="shobot_docking_node",
            name="shobot_docking",
            output="screen",
            parameters=[
                {"cmd_vel_topic": "/cmd_vel"},
                {"odom_topic": "/odom"},
                {"dock_speed": 0.1},
                {"dock_distance": 0.3},
                {"timeout_sec": 30.0},
            ],
        )
    ])
