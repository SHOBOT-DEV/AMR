from pathlib import Path

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_model_path() -> str:
    """Try to locate a bundled YOLO weights file for the default launch argument."""
    try:
        share_dir = Path(get_package_share_directory("shobot_yolo_detection"))
    except PackageNotFoundError:
        share_dir = Path(__file__).resolve().parents[2]

    weights_dir = share_dir / "weights"
    if weights_dir.exists():
        candidates = sorted(weights_dir.glob("*.pt"))
        if candidates:
            return str(candidates[0])
    return str(weights_dir / "yolov10n.pt")


def generate_launch_description() -> LaunchDescription:
    model_arg = DeclareLaunchArgument(
        "model_path",
        default_value=_default_model_path(),
        description="Absolute path to the YOLO weights file (.pt).",
    )
    frame_rate_arg = DeclareLaunchArgument(
        "frame_rate",
        default_value="15.0",
        description="RealSense camera publish rate in Hz.",
    )
    confidence_arg = DeclareLaunchArgument(
        "confidence_threshold",
        default_value="0.4",
        description="Minimum confidence for reporting detections.",
    )

    frame_rate = LaunchConfiguration("frame_rate")
    model_path = LaunchConfiguration("model_path")
    confidence = LaunchConfiguration("confidence_threshold")

    camera_node = Node(
        package="shobot_yolo_detection",
        executable="shobot_realsense_camera_node",
        name="shobot_realsense_camera",
        output="screen",
        parameters=[
            {"frame_rate": frame_rate},
            {"color_topic": "/camera/color/image_raw"},
            {"depth_topic": "/camera/depth/image_raw"},
            {"color_info_topic": "/camera/color/camera_info"},
            {"depth_info_topic": "/camera/depth/camera_info"},
        ],
    )

    detector_node = Node(
        package="shobot_yolo_detection",
        executable="shobot_yolo_detection_node",
        name="shobot_yolo_detector",
        output="screen",
        parameters=[
            {"model_path": model_path},
            {"confidence_threshold": confidence},
            {"color_topic": "/camera/color/image_raw"},
            {"depth_topic": "/camera/depth/image_raw"},
            {"camera_info_topic": "/camera/color/camera_info"},
        ],
    )

    return LaunchDescription(
        [
            model_arg,
            frame_rate_arg,
            confidence_arg,
            camera_node,
            detector_node,
        ]
    )
