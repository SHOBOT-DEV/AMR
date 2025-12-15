from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mission_topic = LaunchConfiguration("mission_topic")
    task_topic = LaunchConfiguration("task_topic")
    status_topic = LaunchConfiguration("status_topic")
    nav_action_name = LaunchConfiguration("nav2_action_name")
    dock_action_name = LaunchConfiguration("dock_action_name")

    return LaunchDescription([
        DeclareLaunchArgument("mission_topic", default_value="/mission_queue", description="Topic for mission batches (JSON)."),
        DeclareLaunchArgument("task_topic", default_value="/task_goal", description="Topic for single PoseStamped tasks."),
        DeclareLaunchArgument("status_topic", default_value="/mission_status", description="Status topic."),
        DeclareLaunchArgument("nav2_action_name", default_value="navigate_to_pose", description="Nav2 action name."),
        DeclareLaunchArgument("dock_action_name", default_value="dock", description="Dock action name."),
        Node(
            package="shobot_mission_control",
            executable="shobot_mission_control_node",
            name="shobot_mission_control",
            output="screen",
            parameters=[
                {"mission_topic": mission_topic},
                {"task_topic": task_topic},
                {"status_topic": status_topic},
                {"nav2_action_name": nav_action_name},
                {"dock_action_name": dock_action_name},
            ],
        )
    ])
