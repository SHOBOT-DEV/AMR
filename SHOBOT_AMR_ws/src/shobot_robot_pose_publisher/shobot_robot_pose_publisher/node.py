#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class RobotPosePublisher(Node):
    """Re-publish pose from odometry as PoseStamped."""

    def __init__(self):
        super().__init__("shobot_robot_pose_publisher")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("pose_topic", "/robot_pose")

        odom_topic = self.get_parameter("odom_topic").value
        pose_topic = self.get_parameter("pose_topic").value

        self.publisher = self.create_publisher(PoseStamped, pose_topic, 10)
        self.subscription = self.create_subscription(
            Odometry, odom_topic, self.callback, 10
        )
        self.get_logger().info(f"Publishing PoseStamped from {odom_topic} to {pose_topic}")

    def callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = RobotPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
