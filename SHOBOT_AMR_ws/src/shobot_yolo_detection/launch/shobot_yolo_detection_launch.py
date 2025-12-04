from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="shobot_yolo_detection",
            executable="shobot_yolo_detection_node",
            name="shobot_yolo_detection",
            output="screen",
        )
    ])
