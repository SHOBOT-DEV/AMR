from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    args = [
        DeclareLaunchArgument("model_path", default_value=""),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("path_topic", default_value="/plan"),
        DeclareLaunchArgument("floor_topic", default_value="/floor_friction"),
        DeclareLaunchArgument("proximity_topic", default_value="/humans/proximity"),
        DeclareLaunchArgument("input_cmd_topic", default_value="/cmd_vel_raw"),
        DeclareLaunchArgument("output_cmd_topic", default_value="/cmd_vel_modulated"),
    ]

    return LaunchDescription(
        args
        + [
            Node(
                package="shobot_rl_agent",
                executable="rl_speed_node",
                name="shobot_rl_speed",
                output="screen",
                parameters=[
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "scan_topic": LaunchConfiguration("scan_topic"),
                        "path_topic": LaunchConfiguration("path_topic"),
                        "floor_topic": LaunchConfiguration("floor_topic"),
                        "proximity_topic": LaunchConfiguration("proximity_topic"),
                        "input_cmd_topic": LaunchConfiguration("input_cmd_topic"),
                        "output_cmd_topic": LaunchConfiguration("output_cmd_topic"),
                    }
                ],
            )
        ]
    )
