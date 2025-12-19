from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "model_path",
            default_value="",
            description="TorchScript model for intent classification (optional).",
        ),
        DeclareLaunchArgument(
            "bbox_topic",
            default_value="/detections",
            description="Camera detection bounding boxes (vision_msgs/Detection2DArray).",
        ),
        DeclareLaunchArgument(
            "scan_topic",
            default_value="/scan",
            description="LiDAR scan topic.",
        ),
        DeclareLaunchArgument(
            "imu_topic",
            default_value="/imu",
            description="IMU topic.",
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/depth/image_raw",
            description="Depth image topic (for motion cues).",
        ),
        DeclareLaunchArgument(
            "decision_topic",
            default_value="/rl_intent/decision",
            description="Output decision topic (std_msgs/String).",
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/rl_intent/cmd_vel",
            description="Output Twist command topic.",
        ),
    ]

    return LaunchDescription(
        args
        + [
            Node(
                package="shobot_rl_agent",
                executable="rl_intent_node",
                name="shobot_rl_intent",
                output="screen",
                parameters=[
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "bbox_topic": LaunchConfiguration("bbox_topic"),
                        "scan_topic": LaunchConfiguration("scan_topic"),
                        "imu_topic": LaunchConfiguration("imu_topic"),
                        "depth_topic": LaunchConfiguration("depth_topic"),
                        "decision_topic": LaunchConfiguration("decision_topic"),
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                    }
                ],
            )
        ]
    )
