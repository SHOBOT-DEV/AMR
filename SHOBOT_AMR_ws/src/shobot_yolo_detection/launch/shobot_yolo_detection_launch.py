from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="shobot_yolo_detection",
            executable="shobot_yolo_detection_node",
            name="depth_detector",
            output="screen",
            parameters=[
                {"detection_topic": "/yolo/detections"},
                {"annotated_topic": "/yolo/detection_image"},
                {"model_path": "yolov10n.pt"},
                {"score_threshold": 0.25},
                {"frame_skip": 5},
                {"use_display": False},
            ],
        )
    ])
