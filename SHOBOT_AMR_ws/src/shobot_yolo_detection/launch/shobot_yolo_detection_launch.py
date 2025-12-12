from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    detection_topic = LaunchConfiguration("detection_topic")
    annotated_topic = LaunchConfiguration("annotated_topic")
    model_path = LaunchConfiguration("model_path")
    score_threshold = LaunchConfiguration("score_threshold")
    frame_skip = LaunchConfiguration("frame_skip")
    use_display = LaunchConfiguration("use_display")
    use_ros_camera = LaunchConfiguration("use_ros_camera")
    color_topic = LaunchConfiguration("color_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    max_detections = LaunchConfiguration("max_detections")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "detection_topic",
                default_value="/detection_info",
                description="Topic for YOLO detections (JSON).",
            ),
            DeclareLaunchArgument(
                "annotated_topic",
                default_value="/yolo/detection_image",
                description="Topic for annotated image output.",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value="yolov10n.pt",
                description="Path to YOLO model file.",
            ),
            DeclareLaunchArgument(
                "score_threshold",
                default_value="0.25",
                description="Confidence threshold for detections.",
            ),
            DeclareLaunchArgument(
                "frame_skip",
                default_value="5",
                description="Process every Nth frame.",
            ),
            DeclareLaunchArgument(
                "max_detections",
                default_value="50",
                description="Cap on number of detections per frame (0 = unlimited).",
            ),
            DeclareLaunchArgument(
                "use_display",
                default_value="False",
                description="Show OpenCV window with detections.",
            ),
            DeclareLaunchArgument(
                "use_ros_camera",
                default_value="False",
                description="Subscribe to ROS topics instead of RealSense capture.",
            ),
            DeclareLaunchArgument(
                "color_topic",
                default_value="/camera/color/image_raw",
                description="Color image topic when use_ros_camera is True.",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/camera/aligned_depth_to_color/image_raw",
                description="Depth image topic when use_ros_camera is True.",
            ),
            Node(
                package="shobot_yolo_detection",
                executable="shobot_yolo_detection_node",
                name="depth_detector",
                output="screen",
                parameters=[
                    {"detection_topic": detection_topic},
                    {"annotated_topic": annotated_topic},
                    {"model_path": model_path},
                    {"score_threshold": score_threshold},
                    {"frame_skip": frame_skip},
                    {"use_display": use_display},
                    {"use_ros_camera": use_ros_camera},
                    {"color_topic": color_topic},
                    {"depth_topic": depth_topic},
                    {"max_detections": max_detections},
                ],
            ),
        ]
    )
