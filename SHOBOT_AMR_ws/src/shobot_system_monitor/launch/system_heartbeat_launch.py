from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    heartbeat_topic = LaunchConfiguration("heartbeat_topic")
    rate_hz = LaunchConfiguration("rate_hz")
    sensor_topics = LaunchConfiguration("sensor_topics")
    node_topics = LaunchConfiguration("node_topics")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "heartbeat_topic",
                default_value="/system/heartbeat",
                description="Topic for the heartbeat JSON string.",
            ),
            DeclareLaunchArgument(
                "rate_hz",
                default_value="1.0",
                description="Publish rate for heartbeat.",
            ),
            DeclareLaunchArgument(
                "sensor_topics",
                default_value="[]",
                description="List of 'name:topic:timeout_sec' strings for sensors.",
            ),
            DeclareLaunchArgument(
                "node_topics",
                default_value="[]",
                description="List of 'name:topic:timeout_sec' strings for nodes.",
            ),
            Node(
                package="shobot_system_monitor",
                executable="system_heartbeat_node",
                name="shobot_system_heartbeat",
                output="screen",
                parameters=[
                    {
                        "heartbeat_topic": heartbeat_topic,
                        "rate_hz": rate_hz,
                        "sensor_topics": sensor_topics,
                        "node_topics": node_topics,
                    }
                ],
            ),
        ]
    )
