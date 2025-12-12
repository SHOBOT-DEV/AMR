from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    publish_rate = LaunchConfiguration("publish_rate")
    timeout_sec = LaunchConfiguration("timeout_sec")
    motor_temp_warn = LaunchConfiguration("motor_temp_warn")
    motor_temp_error = LaunchConfiguration("motor_temp_error")
    network_warn = LaunchConfiguration("network_warn")
    network_error = LaunchConfiguration("network_error")

    return LaunchDescription(
        [
            DeclareLaunchArgument("publish_rate", default_value="1.0"),
            DeclareLaunchArgument("timeout_sec", default_value="2.0"),
            DeclareLaunchArgument("motor_temp_warn", default_value="70.0"),
            DeclareLaunchArgument("motor_temp_error", default_value="85.0"),
            DeclareLaunchArgument("network_warn", default_value="40.0"),
            DeclareLaunchArgument("network_error", default_value="15.0"),
            Node(
                package="shobot_robot_diagnostics",
                executable="robot_diagnostics_node",
                name="shobot_robot_diagnostics",
                output="screen",
                parameters=[
                    {
                        "publish_rate": publish_rate,
                        "timeout_sec": timeout_sec,
                        "motor_temp_warn": motor_temp_warn,
                        "motor_temp_error": motor_temp_error,
                        "network_warn": network_warn,
                        "network_error": network_error,
                    }
                ],
            ),
        ]
    )
