from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="",
        description="Path to a Torch policy .pt file. Leave empty to use heuristic fallback.",
    )
    scan_topic_arg = DeclareLaunchArgument(
        "scan_topic",
        default_value="/scan",
        description="LaserScan topic for observations.",
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/odom",
        description="Odometry topic for observations.",
    )
    cmd_vel_topic_arg = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/cmd_vel",
        description="Twist command topic to publish actions.",
    )

    return LaunchDescription(
        [
            model_path_arg,
            scan_topic_arg,
            odom_topic_arg,
            cmd_vel_topic_arg,
            Node(
                package="shobot_rl_agent",
                executable="rl_agent_node",
                name="shobot_rl_agent",
                output="screen",
                parameters=[
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "scan_topic": LaunchConfiguration("scan_topic"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                    }
                ],
            ),
        ]
    )
