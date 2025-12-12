#!/usr/bin/env python3
"""
Robot Pose Publisher for SHOBOT AMR
---------------------------------------------------------
Converts nav_msgs/Odometry → geometry_msgs/PoseStamped
with safe header, timestamp, and QoS settings.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class RobotPosePublisher(Node):
    """Republish robot pose as PoseStamped for monitoring/visualization."""

    def __init__(self):
        super().__init__("shobot_robot_pose_publisher")

        # ---------------- Parameters ----------------
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("pose_topic", "/robot_pose")
        self.declare_parameter("pose_frame", "")  # "" means keep original

        odom_topic = self.get_parameter("odom_topic").value
        pose_topic = self.get_parameter("pose_topic").value
        self.pose_frame = self.get_parameter("pose_frame").value

        # ---------------- QoS (Sensor Data) ----------------
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        # ---------------- Publisher / Subscriber ----------------
        self.publisher = self.create_publisher(PoseStamped, pose_topic, 10)
        self.create_subscription(Odometry, odom_topic, self.callback, sensor_qos)

        self.get_logger().info(
            f"Re-publishing PoseStamped from {odom_topic} → {pose_topic} "
            f"(frame override: '{self.pose_frame or 'inherit'}')"
        )

    # ------------------------------------------------------------------
    def callback(self, msg: Odometry):
        pose_msg = PoseStamped()

        # Timestamp handling
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            pose_msg.header.stamp = self.get_clock().now().to_msg()
        else:
            pose_msg.header.stamp = msg.header.stamp

        # Frame handling
        pose_msg.header.frame_id = (
            self.pose_frame if self.pose_frame else msg.header.frame_id
        )

        pose_msg.pose = msg.pose.pose

        self.publisher.publish(pose_msg)


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = RobotPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
