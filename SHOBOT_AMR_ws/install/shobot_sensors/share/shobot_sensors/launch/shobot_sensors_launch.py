from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    imu_input = LaunchConfiguration("imu_input_topic")
    imu_output = LaunchConfiguration("imu_output_topic")
    imu_frame = LaunchConfiguration("imu_frame_id")

    left_enc = LaunchConfiguration("left_encoder_topic")
    right_enc = LaunchConfiguration("right_encoder_topic")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheelbase = LaunchConfiguration("wheelbase")
    ticks_per_rev = LaunchConfiguration("ticks_per_rev")
    odom_topic = LaunchConfiguration("odom_topic")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")

    battery_topic = LaunchConfiguration("battery_topic")
    battery_status_topic = LaunchConfiguration("battery_status_topic")
    battery_low_topic = LaunchConfiguration("battery_low_topic")
    battery_low_threshold = LaunchConfiguration("battery_low_threshold")
    battery_critical_threshold = LaunchConfiguration("battery_critical_threshold")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "imu_input_topic",
                default_value="/imu/raw",
                description="Incoming IMU topic (sensor_msgs/Imu).",
            ),
            DeclareLaunchArgument(
                "imu_output_topic",
                default_value="/imu/data",
                description="Output IMU topic after frame/covariance fixup.",
            ),
            DeclareLaunchArgument(
                "imu_frame_id",
                default_value="imu_link",
                description="Frame ID set on IMU messages.",
            ),
            DeclareLaunchArgument(
                "left_encoder_topic",
                default_value="/left_wheel_encoder",
                description="Left wheel encoder count topic (Int32).",
            ),
            DeclareLaunchArgument(
                "right_encoder_topic",
                default_value="/right_wheel_encoder",
                description="Right wheel encoder count topic (Int32).",
            ),
            DeclareLaunchArgument(
                "wheel_radius",
                default_value="0.08",
                description="Wheel radius in meters.",
            ),
            DeclareLaunchArgument(
                "wheelbase",
                default_value="0.447967",
                description="Distance between wheels in meters.",
            ),
            DeclareLaunchArgument(
                "ticks_per_rev",
                default_value="2048",
                description="Encoder ticks per wheel revolution.",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odom/wheel",
                description="Published Odometry topic.",
            ),
            DeclareLaunchArgument(
                "odom_frame",
                default_value="odom",
                description="Odometry frame ID.",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_link",
                description="Base link frame ID.",
            ),
            DeclareLaunchArgument(
                "battery_topic",
                default_value="/battery_state",
                description="Incoming BatteryState topic.",
            ),
            DeclareLaunchArgument(
                "battery_status_topic",
                default_value="/battery_status",
                description="Human-readable battery status topic (String).",
            ),
            DeclareLaunchArgument(
                "battery_low_topic",
                default_value="/battery_low",
                description="Bool topic indicating battery is low/critical.",
            ),
            DeclareLaunchArgument(
                "battery_low_threshold",
                default_value="0.2",
                description="Percentage threshold (0-1) for low battery.",
            ),
            DeclareLaunchArgument(
                "battery_critical_threshold",
                default_value="0.1",
                description="Percentage threshold (0-1) for critical battery.",
            ),
            Node(
                package="shobot_sensors",
                executable="imu_republisher_node",
                name="shobot_imu_republisher",
                output="screen",
                parameters=[
                    {"input_topic": imu_input},
                    {"output_topic": imu_output},
                    {"frame_id": imu_frame},
                ],
            ),
            Node(
                package="shobot_sensors",
                executable="wheel_odometry_node",
                name="shobot_wheel_odometry",
                output="screen",
                parameters=[
                    {"left_encoder_topic": left_enc},
                    {"right_encoder_topic": right_enc},
                    {"wheel_radius": wheel_radius},
                    {"wheelbase": wheelbase},
                    {"ticks_per_rev": ticks_per_rev},
                    {"odom_topic": odom_topic},
                    {"odom_frame": odom_frame},
                    {"base_frame": base_frame},
                ],
            ),
            Node(
                package="shobot_sensors",
                executable="battery_monitor_node",
                name="shobot_battery_monitor",
                output="screen",
                parameters=[
                    {"battery_topic": battery_topic},
                    {"status_topic": battery_status_topic},
                    {"low_topic": battery_low_topic},
                    {"low_threshold": battery_low_threshold},
                    {"critical_threshold": battery_critical_threshold},
                ],
            ),
        ]
    )
