from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    planner_mode = LaunchConfiguration("planner_mode")
    path_goal_topic = LaunchConfiguration("path_goal_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "planner_mode",
            default_value="dwa",
            description="Planner mode: 'dwa' (Nav2) or 'path' (predetermined path to goal)",
        ),
        DeclareLaunchArgument(
            "path_goal_topic",
            default_value="/plan_goal",
            description="Path topic used when planner_mode=='path'; last pose becomes Nav2 goal.",
        ),
        Node(
            package="shobot_navigation",
            executable="shobot_navigation_node",
            name="shobot_navigation",
            output="screen",
            parameters=[
                {"goal_topic": "/goal_pose"},
                {"status_topic": "/navigation_status"},
                {"feedback_topic": "/navigation_feedback"},
                {"nav2_action_name": "navigate_to_pose"},
                {"planner_mode": planner_mode},
                {"path_goal_topic": path_goal_topic},
            ],
        )
    ])
