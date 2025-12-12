from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sources = LaunchConfiguration("sources")
    dock_pose_topic = LaunchConfiguration("dock_pose_topic")
    dock_detected_topic = LaunchConfiguration("dock_detected_topic")
    pose_frame_override = LaunchConfiguration("pose_frame_override")
    publish_status = LaunchConfiguration("publish_status")
    status_topic = LaunchConfiguration("publish_status_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sources",
                default_value="[]",
                description="List of 'name:topic:timeout_sec' PoseStamped detection sources.",
            ),
            DeclareLaunchArgument(
                "dock_pose_topic",
                default_value="/dock_pose",
                description="Output topic for detected dock pose (PoseStamped).",
            ),
            DeclareLaunchArgument(
                "dock_detected_topic",
                default_value="/dock_detected",
                description="Output topic for detection flag (Bool).",
            ),
            DeclareLaunchArgument(
                "pose_frame_override",
                default_value="",
                description="If non-empty, overrides frame_id on dock_pose.",
            ),
            DeclareLaunchArgument(
                "publish_status",
                default_value="true",
                description="Publish JSON status summary on publish_status_topic.",
            ),
            DeclareLaunchArgument(
                "publish_status_topic",
                default_value="/dock_detection_status",
                description="Status topic for debug/monitoring.",
            ),
            Node(
                package="shobot_dock_detection",
                executable="dock_detection_node",
                name="shobot_dock_detection",
                output="screen",
                parameters=[
                    {
                        "sources": sources,
                        "dock_pose_topic": dock_pose_topic,
                        "dock_detected_topic": dock_detected_topic,
                        "pose_frame_override": pose_frame_override,
                        "publish_status": publish_status,
                        "publish_status_topic": status_topic,
                    }
                ],
            ),
        ]
    )
