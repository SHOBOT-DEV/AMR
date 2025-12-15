#!/usr/bin/env python3
"""
Republish IMU data with fixed frame_id, timestamp update,
and optional covariance. Includes TF broadcasting.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class ImuRepublisher(Node):
    """Ensure IMU messages have a consistent frame_id and covariance."""

    def __init__(self):
        super().__init__("shobot_imu_republisher")

        # ---------------- Parameters ----------------
        self.declare_parameter("input_topic", "/imu/raw")
        self.declare_parameter("output_topic", "/imu/data")
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("orientation_covariance", [])
        self.declare_parameter("angular_velocity_covariance", [])
        self.declare_parameter("linear_acceleration_covariance", [])
        self.declare_parameter("publish_tf", True)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        # Covariance extraction
        self.orientation_cov = self._get_cov("orientation_covariance")
        self.angular_vel_cov = self._get_cov("angular_velocity_covariance")
        self.linear_acc_cov = self._get_cov("linear_acceleration_covariance")

        # ---------------- Publishers / Subscribers ----------------
        self.pub = self.create_publisher(Imu, output_topic, 10)
        self.create_subscription(Imu, input_topic, self.imu_cb, 50)

        # TF broadcaster (optional)
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f"Republishing IMU {input_topic} -> {output_topic} "
            f"with frame_id={self.frame_id}"
        )

    # ----------------------------------------------------------------------
    def _get_cov(self, name):
        """Validate covariance (must be length 9 or empty)."""
        cov = self.get_parameter(name).value
        if cov and len(cov) != 9:
            self.get_logger().warn(f"Parameter '{name}' must have 9 values. Ignoring it.")
            return None
        return cov if cov else None

    # ----------------------------------------------------------------------
    def imu_cb(self, msg: Imu):
        """Republish IMU with fixed frame, timestamp, and covariance."""

        new_msg = Imu()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = self.frame_id

        # Copy values
        new_msg.orientation = msg.orientation
        new_msg.angular_velocity = msg.angular_velocity
        new_msg.linear_acceleration = msg.linear_acceleration

        # Covariances (fallback to '-1' = unknown)
        new_msg.orientation_covariance = (
            list(self.orientation_cov) if self.orientation_cov else [-1.0] * 9
        )
        new_msg.angular_velocity_covariance = (
            list(self.angular_vel_cov) if self.angular_vel_cov else [-1.0] * 9
        )
        new_msg.linear_acceleration_covariance = (
            list(self.linear_acc_cov) if self.linear_acc_cov else [-1.0] * 9
        )

        # Publish IMU message
        self.pub.publish(new_msg)

        # Publish TF if required
        if self.publish_tf:
            self.publish_tf_transform()

    # ----------------------------------------------------------------------
    def publish_tf_transform(self):
        """Broadcast static transform from base_link → imu_link."""
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.base_frame
        tf.child_frame_id = self.frame_id

        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0

        tf.transform.rotation.w = 1.0  # No rotation

        self.tf_broadcaster.sendTransform(tf)


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ImuRepublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
