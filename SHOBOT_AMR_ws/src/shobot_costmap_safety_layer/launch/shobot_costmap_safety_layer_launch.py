from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scan_topic = LaunchConfiguration("scan_topic")
    safety_topic = LaunchConfiguration("safety_stop_topic")
    threshold = LaunchConfiguration("threshold")
    landmark_topic = LaunchConfiguration("landmark_topic")
    landmark_stop_radius = LaunchConfiguration("landmark_stop_radius")

    return LaunchDescription([
        DeclareLaunchArgument("scan_topic", default_value="/scan", description="LaserScan topic."),
        DeclareLaunchArgument("safety_stop_topic", default_value="/safety_stop", description="Output safety stop Bool topic."),
        DeclareLaunchArgument("threshold", default_value="0.4", description="Min range before stopping."),
        DeclareLaunchArgument("landmark_topic", default_value="", description="Optional PoseArray landmarks to stop near."),
        DeclareLaunchArgument("landmark_stop_radius", default_value="0.5", description="Stop if landmark is within this radius (m)."),
        Node(
            package="shobot_costmap_safety_layer",
            executable="shobot_costmap_safety_layer_node",
            name="shobot_costmap_safety_layer",
            output="screen",
            parameters=[
                {"scan_topic": scan_topic},
                {"safety_stop_topic": safety_topic},
                {"threshold": threshold},
                {"landmark_topic": landmark_topic},
                {"landmark_stop_radius": landmark_stop_radius},
            ],
        )
    ])
