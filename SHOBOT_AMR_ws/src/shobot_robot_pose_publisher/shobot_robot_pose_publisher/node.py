#!/usr/bin/env python3
"""
Robot Pose Publisher for SHOBOT AMR
=========================================================
Converts nav_msgs/Odometry → geometry_msgs/PoseStamped
with safe header handling and sensor-friendly QoS.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class RobotPosePublisher(Node):
    """Republish robot pose as PoseStamped for monitoring and visualization."""

    def __init__(self):
        super().__init__("shobot_robot_pose_publisher")

        # ---------------- Parameters ----------------
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("pose_topic", "/robot_pose")
        self.declare_parameter("pose_frame", "")  # Empty → inherit from odom

        odom_topic = self.get_parameter("odom_topic").value
        pose_topic = self.get_parameter("pose_topic").value
        self.pose_frame = self.get_parameter("pose_frame").value.strip()

        # ---------------- QoS (Sensor Data) ----------------
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        # ---------------- Publisher / Subscriber ----------------
        self.publisher = self.create_publisher(
            PoseStamped, pose_topic, sensor_qos
        )
        self.create_subscription(
            Odometry, odom_topic, self.odom_cb, sensor_qos
        )

        self.get_logger().info(
            f"RobotPosePublisher active\n"
            f"  Odometry : {odom_topic}\n"
            f"  Pose     : {pose_topic}\n"
            f"  Frame    : {self.pose_frame or 'inherit'}"
        )

    # ------------------------------------------------------------------
    def odom_cb(self, msg: Odometry):
        pose_msg = PoseStamped()

        # Timestamp: inherit if valid, else use current time
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            pose_msg.header.stamp = self.get_clock().now().to_msg()
        else:
            pose_msg.header.stamp = msg.header.stamp

        # Frame: override or inherit
        pose_msg.header.frame_id = (
            self.pose_frame if self.pose_frame else msg.header.frame_id
        )

        pose_msg.pose = msg.pose.pose
        self.publisher.publish(pose_msg)


# ======================================================================
def main(args=None):
    rclpy.init(args=args)
    node = RobotPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
