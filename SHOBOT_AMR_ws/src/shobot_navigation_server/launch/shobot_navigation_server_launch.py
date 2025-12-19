from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def include_launch(pkg, file, args=None):
    args = args or {}
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory(pkg) + f"/launch/{file}"
        ),
        launch_arguments=args.items(),
    )


def generate_launch_description():
    bringup_nav2 = LaunchConfiguration("bringup_nav2")
    nav2_params = LaunchConfiguration("nav2_params_file")
    map_yaml = LaunchConfiguration("map_yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bringup_nav2",
                default_value="false",
                description="If true, include nav2_bringup/bringup_launch.py with provided params/map.",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value="",
                description="Path to Nav2 params YAML (required if bringup_nav2:=true).",
            ),
            DeclareLaunchArgument(
                "map_yaml",
                default_value="",
                description="Path to map YAML for Nav2 (required if bringup_nav2:=true and not using SLAM).",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time for all nodes.",
            ),
            # Optional Nav2 bringup (requires parameters provided)
            GroupAction(
                condition=IfCondition(bringup_nav2),
                actions=[
                    include_launch(
                        "nav2_bringup",
                        "bringup_launch.py",
                        {
                            "use_sim_time": use_sim_time,
                            "params_file": nav2_params,
                            "map": map_yaml,
                        },
                    )
                ],
            ),
            # Navigation client (waits for Nav2 action server)
            include_launch("shobot_navigation", "shobot_navigation_launch.py"),
            Node(
                package="shobot_navigation_server",
                executable="shobot_navigation_server_node",
                name="shobot_navigation_server",
                output="screen",
                parameters=[
                    {"nav2_action_name": "navigate_to_pose"},
                    {"status_topic": "/navigation_server/status"},
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )
