#!/usr/bin/env python3
"""
IMU Republisher for SHOBOT
=====================================================
- Normalizes IMU frame_id
- Updates timestamp
- Applies optional covariance
- Publishes optional static TF (base_link → imu_link)
"""

from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


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

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        # ---------------- Covariances ----------------
        self.orientation_cov = self._get_cov("orientation_covariance")
        self.angular_vel_cov = self._get_cov("angular_velocity_covariance")
        self.linear_acc_cov = self._get_cov("linear_acceleration_covariance")

        # ---------------- Publisher / Subscriber ----------------
        self.pub = self.create_publisher(Imu, self.output_topic, 10)
        self.create_subscription(
            Imu,
            self.input_topic,
            self.imu_cb,
            qos_profile_sensor_data,
        )

        # ---------------- Static TF ----------------
        if self.publish_tf:
            self.tf_broadcaster = StaticTransformBroadcaster(self)
            self._publish_static_tf()

        self.get_logger().info(
            f"IMU republisher active\n"
            f"Input topic  : {self.input_topic}\n"
            f"Output topic : {self.output_topic}\n"
            f"Frame ID     : {self.frame_id}\n"
            f"Publish TF   : {self.publish_tf}"
        )

    # ------------------------------------------------------------------
    def _get_cov(self, name: str) -> Optional[List[float]]:
        """Validate covariance parameter (must be length 9)."""
        raw = self.get_parameter(name).value
        if not raw:
            return None

        try:
            cov = [float(v) for v in raw]
        except (TypeError, ValueError):
            self.get_logger().warn(
                f"Parameter '{name}' contains non-numeric values; ignoring."
            )
            return None

        if len(cov) != 9:
            self.get_logger().warn(
                f"Parameter '{name}' must have exactly 9 values; ignoring."
            )
            return None

        return cov

    # ------------------------------------------------------------------
    def imu_cb(self, msg: Imu):
        """Republish IMU with fixed frame, timestamp, and covariance."""

        out = Imu()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id

        # Copy measurements
        out.orientation = msg.orientation
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration

        # Apply covariances (ROS convention: -1 = unknown)
        out.orientation_covariance = (
            self.orientation_cov if self.orientation_cov else [-1.0] * 9
        )
        out.angular_velocity_covariance = (
            self.angular_vel_cov if self.angular_vel_cov else [-1.0] * 9
        )
        out.linear_acceleration_covariance = (
            self.linear_acc_cov if self.linear_acc_cov else [-1.0] * 9
        )

        self.pub.publish(out)

    # ------------------------------------------------------------------
    def _publish_static_tf(self):
        """Publish static transform base_link → imu_link."""
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.base_frame
        tf.child_frame_id = self.frame_id

        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0

        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tf)


# ======================================================================
def main(args=None):
    rclpy.init(args=args)
    node = ImuRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
