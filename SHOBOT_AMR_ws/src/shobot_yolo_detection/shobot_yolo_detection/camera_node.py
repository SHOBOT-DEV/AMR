#!/usr/bin/env python3
"""
Camera Node
===================================================
Publishes RealSense color + depth frames as ROS2 topics.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import cv2


class RealSenseCameraNode(Node):
    def __init__(self):
        super().__init__("realsense_camera_node")

        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")

        self.color_topic = self.get_parameter("color_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value

        self.color_pub = self.create_publisher(Image, self.color_topic, 10)
        self.depth_pub = self.create_publisher(Image, self.depth_topic, 10)

        self.bridge = CvBridge()

        # RealSense Setup
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        self.align = rs.align(rs.stream.color)
        self.pipeline.start(config)

        self.timer = self.create_timer(0.03, self.capture_frame)

        self.get_logger().info(
            f"Camera Node started → publishing: {self.color_topic}, {self.depth_topic}"
        )

    def capture_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)

        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        color_img = np.asanyarray(color_frame.get_data())
        depth_img = np.asanyarray(depth_frame.get_data())

        self.color_pub.publish(self.bridge.cv2_to_imgmsg(color_img, "bgr8"))
        self.depth_pub.publish(self.bridge.cv2_to_imgmsg(depth_img, "passthrough"))

    def destroy_node(self):
        try:
            self.pipeline.stop()
            cv2.destroyAllWindows()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
