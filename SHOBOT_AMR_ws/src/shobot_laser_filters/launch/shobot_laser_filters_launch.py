from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="shobot_laser_filters",
            executable="shobot_laser_filters_node",
            name="shobot_laser_filters",
            output="screen",
            parameters=[
                {"input_topic": "/scan"},
                {"output_topic": "/scan_filtered"},
                {"range_min": 0.05},
                {"range_max": 10.0},
                {"replace_invalid_with": 0.0},
            ],
        )
    ])
