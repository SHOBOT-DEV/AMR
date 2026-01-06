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
        # Teleop keyboard publisher (optional; runs in terminal)
        Node(
            package="shobot_teleop",
            executable="teleop_key",
            name="shobot_teleop",
            output="screen",
            parameters=[{"teleop_topic": "/cmd_vel/teleop"}],
        ),
        # Twist multiplexer: chooses between safety, teleop, nav
        Node(
            package="shobot_twist_mux",
            executable="shobot_twist_mux_node",
            name="shobot_twist_mux",
            output="screen",
            parameters=[
                {"sources": ["safety", "teleop", "nav"]},
                {"priorities": {"safety": 100, "teleop": 50, "nav": 10}},
                {"timeouts": {"safety": 0.5, "teleop": 0.5, "nav": 1.0}},
                {
                    "topics": {
                        "safety": "/cmd_vel/safety",
                        "teleop": "/cmd_vel/teleop",
                        "nav": "/cmd_vel/nav",
                    }
                },
                {"output_topic": "/cmd_vel_raw"},
                {"safety_stop_topic": "/safety_stop"},
                {"rate_hz": 20.0},
            ],
        ),
        # Trajectory controller to smooth cmd_vel before motors
        Node(
            package="shobot_trajectory_controller",
            executable="shobot_trajectory_controller_node",
            name="shobot_trajectory_controller",
            output="screen",
            parameters=[
                {"input_topic": "/cmd_vel_raw"},
                {"output_topic": "/cmd_vel"},
                {"rate_hz": 30.0},
                {"max_linear_accel": 0.5},
                {"max_angular_accel": 1.0},
            ],
        ),
        # Safety layer from LaserScan to /safety_stop + safety cmd_vel if desired
        Node(
            package="shobot_costmap_safety_layer",
            executable="shobot_costmap_safety_layer_node",
            name="shobot_costmap_safety_layer",
            output="screen",
            parameters=[
                {"scan_topic": "/scan"},
                {"safety_stop_topic": "/safety_stop"},
                {"threshold": 0.4},
            ],
        ),
        # Laser filter (optional, upstream)
        Node(
            package="shobot_laser_filters",
            executable="shobot_laser_filters_node",
            name="shobot_laser_filters",
            output="screen",
            parameters=[
                {"input_topic": "/scan"},
                {"output_topic": "/scan_filtered"},
                {"range_min": 0.05},
                {"range_max": 10.0},
                {"replace_invalid_with": 0.0},
            ],
        ),
        # Navigation client (Nav2 goal interface) publishing nav cmd_vel
        Node(
            package="shobot_navigation",
            executable="shobot_navigation_node",
            name="shobot_navigation",
            output="screen",
            parameters=[
                {"goal_topic": "/goal_pose"},
                {"status_topic": "/navigation_status"},
                {"feedback_topic": "/navigation_feedback"},
                {"nav2_action_name": "/navigate_to_pose"},
                {"planner_mode": planner_mode},
                {"path_goal_topic": path_goal_topic},
            ],
            remappings=[("/cmd_vel", "/cmd_vel/nav")],
        ),
        # YOLO detection (optional; leave disabled if not needed)
        Node(
            package="shobot_yolo_detection",
            executable="shobot_yolo_detection_node",
            name="shobot_yolo_detection",
            output="screen",
            parameters=[
                {"model_path": "yolov10n.pt"},
                {"detection_topic": "/detection_info"},
                {"annotated_topic": "/yolo/detection_image"},
                {"use_ros_camera": False},
                {"score_threshold": 0.25},
                {"frame_skip": 5},
            ],
        ),
    ])
