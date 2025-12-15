from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output_dir = LaunchConfiguration("output_dir")
    max_file_size_mb = LaunchConfiguration("max_file_size_mb")
    playback_rate = LaunchConfiguration("playback_rate")
    sensor_topics = LaunchConfiguration("sensor_topics")
    mission_topic = LaunchConfiguration("mission_topic")
    task_failure_topic = LaunchConfiguration("task_failure_topic")
    error_topic = LaunchConfiguration("error_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("output_dir", default_value="~/.shobot/log_recorder"),
            DeclareLaunchArgument("max_file_size_mb", default_value="200.0"),
            DeclareLaunchArgument("playback_rate", default_value="1.0"),
            DeclareLaunchArgument("sensor_topics", default_value="[]"),
            DeclareLaunchArgument("mission_topic", default_value="/mission_status"),
            DeclareLaunchArgument("task_failure_topic", default_value="/task_failures"),
            DeclareLaunchArgument("error_topic", default_value="/system/errors"),
            Node(
                package="shobot_log_recorder",
                executable="log_recorder_node",
                name="shobot_log_recorder",
                output="screen",
                parameters=[
                    {
                        "output_dir": output_dir,
                        "max_file_size_mb": max_file_size_mb,
                        "playback_rate": playback_rate,
                        "sensor_topics": sensor_topics,
                        "mission_topic": mission_topic,
                        "task_failure_topic": task_failure_topic,
                        "error_topic": error_topic,
                    }
                ],
            ),
        ]
    )
