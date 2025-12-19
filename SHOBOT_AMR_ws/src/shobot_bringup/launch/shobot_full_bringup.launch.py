from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def include_pkg_launch(package: str, launch_file: str, args: dict = None):
    args = args or {}
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory(package) + f"/launch/{launch_file}"
        ),
        launch_arguments={k: str(v) for k, v in args.items()}.items(),
    )


def generate_launch_description():
    use_rl = LaunchConfiguration("use_rl")
    use_api_bridge = LaunchConfiguration("use_api_bridge")
    use_nav_server = LaunchConfiguration("use_nav_server")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rl",
                default_value="false",
                description="Start RL nodes (intent + speed + base policy).",
            ),
            DeclareLaunchArgument(
                "use_api_bridge",
                default_value="true",
                description="Start API bridge node.",
            ),
            DeclareLaunchArgument(
                "use_nav_server",
                default_value="true",
                description="Start navigation server wrapper.",
            ),
            GroupAction(
                actions=[
                    include_pkg_launch(
                        "shobot_tf_static", "static_tf_publisher_launch.py"
                    ),
                    include_pkg_launch("shobot_sensors", "shobot_sensors_launch.py"),
                    include_pkg_launch(
                        "shobot_laser_filters", "shobot_laser_filters_launch.py"
                    ),
                    include_pkg_launch(
                        "shobot_costmap_safety_layer",
                        "shobot_costmap_safety_layer_launch.py",
                    ),
                    include_pkg_launch("shobot_twist_mux", "shobot_twist_mux_launch.py"),
                    include_pkg_launch(
                        "shobot_navigation", "shobot_navigation_launch.py"
                    ),
                    GroupAction(
                        condition=IfCondition(use_nav_server),
                        actions=[
                            include_pkg_launch(
                                "shobot_navigation_server",
                                "shobot_navigation_server_launch.py",
                                {"use_sim_time": "false"},
                            )
                        ],
                    ),
                    GroupAction(
                        condition=IfCondition(use_api_bridge),
                        actions=[
                            include_pkg_launch(
                                "shobot_api_bridge",
                                "api_bridge_launch.py",
                                {"use_sim_time": "false"},
                            )
                        ],
                    ),
                    GroupAction(
                        condition=IfCondition(use_rl),
                        actions=[
                            include_pkg_launch(
                                "shobot_rl_agent", "shobot_rl_intent_launch.py"
                            ),
                            include_pkg_launch(
                                "shobot_rl_agent", "shobot_rl_speed_launch.py"
                            ),
                            include_pkg_launch(
                                "shobot_rl_agent", "shobot_rl_agent_launch.py"
                            ),
                        ],
                    ),
                ]
            ),
        ]
    )
