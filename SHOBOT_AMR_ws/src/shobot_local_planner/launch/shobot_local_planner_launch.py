from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="shobot_local_planner",
            executable="shobot_local_planner_node",
            name="shobot_local_planner",
            output="screen",
            parameters=[
                {"path_topic": "/plan"},
                {"cmd_vel_topic": "/cmd_vel"},
                {"odom_topic": "/odom"},
                {"lookahead_distance": 0.6},
                {"max_linear": 0.4},
                {"max_angular": 1.2},
            ],
        )
    ])
