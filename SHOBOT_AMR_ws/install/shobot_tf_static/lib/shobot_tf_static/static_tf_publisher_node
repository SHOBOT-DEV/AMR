#!/usr/bin/env python3
"""
Publish required static transforms for SHOBOT.

Default transforms (edit via parameters/YAML):
- base_link -> laser
- base_link -> imu
- base_link -> camera_frame
- map -> odom (optional, off by default to avoid duplicating Nav2's tree)
"""

import math
from collections.abc import Sequence
from typing import List

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class StaticTFPublisher(Node):
    """Static transform publisher with parameterized frames/offsets."""

    def __init__(self):
        super().__init__("shobot_static_tf_publisher")

        # Frame IDs
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("laser_frame", "laser")
        self.declare_parameter("imu_frame", "imu")
        self.declare_parameter("camera_frame", "camera_frame")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")

        # Translations / rotations (roll, pitch, yaw in radians)
        self.declare_parameter("base_to_laser_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("base_to_laser_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("base_to_imu_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("base_to_imu_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("base_to_camera_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("base_to_camera_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("map_to_odom_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("map_to_odom_rpy", [0.0, 0.0, 0.0])

        # Optional map -> odom (leave false when Nav2 publishes this)
        self.declare_parameter("publish_map_to_odom", False)

        self.broadcaster = StaticTransformBroadcaster(self)
        transforms = self._build_transforms()
        self.broadcaster.sendTransform(transforms)
        self._log_transforms(transforms)

    # ------------------------------------------------------------------ #
    def _build_transforms(self) -> List[TransformStamped]:
        transforms: List[TransformStamped] = []
        base_frame = self.get_parameter("base_frame").value

        transforms.append(
            self._make_transform(
                base_frame,
                self.get_parameter("laser_frame").value,
                self._get_vector("base_to_laser_translation", 3),
                self._get_vector("base_to_laser_rpy", 3),
            )
        )
        transforms.append(
            self._make_transform(
                base_frame,
                self.get_parameter("imu_frame").value,
                self._get_vector("base_to_imu_translation", 3),
                self._get_vector("base_to_imu_rpy", 3),
            )
        )
        transforms.append(
            self._make_transform(
                base_frame,
                self.get_parameter("camera_frame").value,
                self._get_vector("base_to_camera_translation", 3),
                self._get_vector("base_to_camera_rpy", 3),
            )
        )

        if self.get_parameter("publish_map_to_odom").value:
            transforms.append(
                self._make_transform(
                    self.get_parameter("map_frame").value,
                    self.get_parameter("odom_frame").value,
                    self._get_vector("map_to_odom_translation", 3),
                    self._get_vector("map_to_odom_rpy", 3),
                )
            )

        return transforms

    def _make_transform(
        self,
        parent: str,
        child: str,
        translation: List[float],
        rpy: List[float],
    ) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent
        transform.child_frame_id = child

        transform.transform.translation.x = float(translation[0])
        transform.transform.translation.y = float(translation[1])
        transform.transform.translation.z = float(translation[2])

        qx, qy, qz, qw = self._rpy_to_quaternion(
            float(rpy[0]), float(rpy[1]), float(rpy[2])
        )
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        return transform

    def _get_vector(self, name: str, expected_len: int) -> List[float]:
        raw = self.get_parameter(name).value
        if isinstance(raw, str) or not isinstance(raw, Sequence):
            self.get_logger().warn(
                f"Parameter '{name}' must be a sequence of length {expected_len}; "
                "using zeros."
            )
            return [0.0] * expected_len

        values = list(raw)
        if len(values) != expected_len:
            self.get_logger().warn(
                f"Parameter '{name}' expected length {expected_len}, "
                f"got {len(values)}; truncating/padding with zeros."
            )
            values = (values + [0.0] * expected_len)[:expected_len]

        try:
            return [float(v) for v in values]
        except (TypeError, ValueError):
            self.get_logger().warn(
                f"Parameter '{name}' could not be converted to float; using zeros."
            )
            return [0.0] * expected_len

    @staticmethod
    def _rpy_to_quaternion(roll: float, pitch: float, yaw: float):
        """Convert roll/pitch/yaw to quaternion (x, y, z, w)."""
        half_roll = roll * 0.5
        half_pitch = pitch * 0.5
        half_yaw = yaw * 0.5

        sin_r = math.sin(half_roll)
        cos_r = math.cos(half_roll)
        sin_p = math.sin(half_pitch)
        cos_p = math.cos(half_pitch)
        sin_y = math.sin(half_yaw)
        cos_y = math.cos(half_yaw)

        x = sin_r * cos_p * cos_y - cos_r * sin_p * sin_y
        y = cos_r * sin_p * cos_y + sin_r * cos_p * sin_y
        z = cos_r * cos_p * sin_y - sin_r * sin_p * cos_y
        w = cos_r * cos_p * cos_y + sin_r * sin_p * sin_y
        return x, y, z, w

    def _log_transforms(self, transforms: List[TransformStamped]):
        for tf in transforms:
            t = tf.transform.translation
            r = tf.transform.rotation
            self.get_logger().info(
                f"Static TF: {tf.header.frame_id} -> {tf.child_frame_id} | "
                f"xyz=({t.x:.3f}, {t.y:.3f}, {t.z:.3f}) "
                f"quat=({r.x:.3f}, {r.y:.3f}, {r.z:.3f}, {r.w:.3f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = StaticTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
