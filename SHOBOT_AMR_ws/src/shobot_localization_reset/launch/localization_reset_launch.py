from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    initialpose_topic = LaunchConfiguration("initialpose_topic")
    reset_pose_topic = LaunchConfiguration("reset_pose_topic")
    frame_id = LaunchConfiguration("frame_id")
    default_pose_xy_yaw = LaunchConfiguration("default_pose_xy_yaw")
    cov_x = LaunchConfiguration("covariance_x")
    cov_y = LaunchConfiguration("covariance_y")
    cov_yaw = LaunchConfiguration("covariance_yaw")
    service_name = LaunchConfiguration("service_name")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "initialpose_topic",
                default_value="/initialpose",
                description="Output topic for pose initialization (AMCL).",
            ),
            DeclareLaunchArgument(
                "reset_pose_topic",
                default_value="/reset_initialpose",
                description="Input topic to relay PoseWithCovarianceStamped resets.",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="map",
                description="Frame for /initialpose messages.",
            ),
            DeclareLaunchArgument(
                "default_pose_xy_yaw",
                default_value="[0.0, 0.0, 0.0]",
                description="Default x,y,yaw (radians) used when calling Trigger service.",
            ),
            DeclareLaunchArgument(
                "covariance_x",
                default_value="0.25",
                description="Covariance on x (m^2) for default pose.",
            ),
            DeclareLaunchArgument(
                "covariance_y",
                default_value="0.25",
                description="Covariance on y (m^2) for default pose.",
            ),
            DeclareLaunchArgument(
                "covariance_yaw",
                default_value=str((3.141592653589793 / 18) ** 2),  # ~10 deg^2
                description="Covariance on yaw (rad^2) for default pose.",
            ),
            DeclareLaunchArgument(
                "service_name",
                default_value="reset_localization",
                description="Trigger service that publishes the default pose.",
            ),
            Node(
                package="shobot_localization_reset",
                executable="localization_reset_node",
                name="shobot_localization_reset",
                output="screen",
                parameters=[
                    {
                        "initialpose_topic": initialpose_topic,
                        "reset_pose_topic": reset_pose_topic,
                        "frame_id": frame_id,
                        "default_pose_xy_yaw": default_pose_xy_yaw,
                        "covariance_x": cov_x,
                        "covariance_y": cov_y,
                        "covariance_yaw": cov_yaw,
                        "service_name": service_name,
                    }
                ],
            ),
        ]
    )
