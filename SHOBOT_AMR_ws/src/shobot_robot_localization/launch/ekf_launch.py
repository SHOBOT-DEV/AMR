from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    base_odom = LaunchConfiguration("base_odom_topic")
    wheel_odom = LaunchConfiguration("wheel_odom_topic")
    visual_slam_odom = LaunchConfiguration("visual_slam_topic")
    lidar_odom = LaunchConfiguration("lidar_odom_topic")
    camera_vo = LaunchConfiguration("camera_vo_topic")
    imu_topic = LaunchConfiguration("imu_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "base_odom_topic",
                default_value="/odom",
                description="Base odometry source (e.g., fused wheel odom).",
            ),
            DeclareLaunchArgument(
                "wheel_odom_topic",
                default_value="/odom/wheel",
                description="Wheel encoder odometry source.",
            ),
            DeclareLaunchArgument(
                "visual_slam_topic",
                default_value="/rtabmap/odom",
                description="Visual SLAM odometry (e.g., RTAB-Map).",
            ),
            DeclareLaunchArgument(
                "lidar_odom_topic",
                default_value="/lidar_odom",
                description="LiDAR odometry / scan matching topic.",
            ),
            DeclareLaunchArgument(
                "camera_vo_topic",
                default_value="/vo",
                description="Camera visual odometry topic.",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/imu/data",
                description="IMU topic.",
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="shobot_ekf",
                output="screen",
                parameters=[
                    "config/ekf.yaml",
                    {
                        "odom0": base_odom,
                        "odom1": wheel_odom,
                        "odom2": visual_slam_odom,
                        "odom3": lidar_odom,
                        "odom4": camera_vo,
                        "imu0": imu_topic,
                    },
                ],
            )
        ]
    )
