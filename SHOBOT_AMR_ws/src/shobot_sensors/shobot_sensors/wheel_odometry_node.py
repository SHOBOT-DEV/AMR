#!/usr/bin/env python3
"""
Differential Drive Wheel Odometry for SHOBOT
=======================================================
Computes /odom/wheel from encoder ticks with TF broadcast.
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Twist, Vector3, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

from tf2_ros import TransformBroadcaster


class WheelOdometry(Node):

    def __init__(self):
        super().__init__("shobot_wheel_odometry")

        # ---------------- Parameters ----------------
        self.declare_parameter("left_encoder_topic", "/left_wheel_encoder")
        self.declare_parameter("right_encoder_topic", "/right_wheel_encoder")
        self.declare_parameter("wheel_radius", 0.08)
        self.declare_parameter("wheelbase", 0.447967)
        self.declare_parameter("ticks_per_rev", 2048)
        self.declare_parameter("odom_topic", "/odom/wheel")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.left_topic = self.get_parameter("left_encoder_topic").value
        self.right_topic = self.get_parameter("right_encoder_topic").value
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.ticks_per_rev = float(self.get_parameter("ticks_per_rev").value)
        self.odom_topic = self.get_parameter("odom_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        # ---------------- Encoder State ----------------
        self.left_prev: Optional[int] = None
        self.right_prev: Optional[int] = None
        self.left_curr: Optional[int] = None
        self.right_curr: Optional[int] = None

        self.last_time = self.get_clock().now()

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_pub = TransformBroadcaster(self)

        # Subscriptions
        self.create_subscription(Int32, self.left_topic, self.left_cb, 50)
        self.create_subscription(Int32, self.right_topic, self.right_cb, 50)

        # Timer (50 Hz)
        self.create_timer(0.02, self.update_odom)

        self.get_logger().info(
            f"Wheel odometry started using encoders: {self.left_topic}, {self.right_topic}"
        )

    # ------------------------------------------------------------------
    def left_cb(self, msg: Int32):
        self.left_curr = msg.data
        if self.left_prev is None:
            self.left_prev = msg.data

    def right_cb(self, msg: Int32):
        self.right_curr = msg.data
        if self.right_prev is None:
            self.right_prev = msg.data

    # ------------------------------------------------------------------
    def _delta_ticks(self, prev: int, curr: int) -> float:
        """Handle encoder overflow for unsigned 32-bit or signed values."""
        MAX_INT32 = 2**31 - 1
        MIN_INT32 = -2**31

        delta = curr - prev

        # Handle unsigned overflow
        if delta > MAX_INT32:
            delta -= (2**32)
        elif delta < -MAX_INT32:
            delta += (2**32)

        return float(delta)

    # ------------------------------------------------------------------
    def update_odom(self):
        if self.left_prev is None or self.right_prev is None:
            return
        if self.left_curr is None or self.right_curr is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return

        d_left_ticks = self._delta_ticks(self.left_prev, self.left_curr)
        d_right_ticks = self._delta_ticks(self.right_prev, self.right_curr)

        self.left_prev = self.left_curr
        self.right_prev = self.right_curr
        self.last_time = now

        # Convert ticks → meters
        meters_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
        d_left = d_left_ticks * meters_per_tick
        d_right = d_right_ticks * meters_per_tick

        # Differential drive pose update
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheelbase

        self.yaw = self._normalize_angle(self.yaw + d_theta)
        self.x += d_center * math.cos(self.yaw)
        self.y += d_center * math.sin(self.yaw)

        # ---------------- Publish Odometry ----------------
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = self._quat_from_yaw(self.yaw)

        odom.twist.twist = Twist(
            linear=Vector3(x=d_center / dt, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=d_theta / dt),
        )

        self.odom_pub.publish(odom)

        # ---------------- Publish TF ----------------
        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation = self._quat_from_yaw(self.yaw)

        self.tf_pub.sendTransform(tf)

    # ------------------------------------------------------------------
    @staticmethod
    def _quat_from_yaw(yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    @staticmethod
    def _normalize_angle(a: float) -> float:
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
