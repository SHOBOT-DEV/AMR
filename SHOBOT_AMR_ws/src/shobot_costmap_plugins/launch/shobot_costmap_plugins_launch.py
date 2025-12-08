from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="shobot_costmap_plugins",
            executable="shobot_costmap_plugins_node",
            name="shobot_costmap_plugins",
            output="screen",
            parameters=[
                {"frame_id": "map"},
                {"resolution": 0.1},
                {"width": 20.0},
                {"height": 20.0},
                {"origin_x": -10.0},
                {"origin_y": -10.0},
                {"obstacle_topic": "/obstacles"},
                {"costmap_topic": "/shobot_costmap"},
                {"max_obstacle_height": 2.0},
                {"min_obstacle_height": 0.05},
                {"inflation_radius": 0.2},
            ],
        )
    ])
