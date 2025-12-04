#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2


class PointCloudToLaserScanNode(Node):
    """Minimal placeholder that emits an empty LaserScan for each PointCloud2 received."""

    def __init__(self):
        super().__init__("shobot_pointcloud_to_laserscan")
        self.declare_parameter("input_topic", "/points")
        self.declare_parameter("output_topic", "/scan_from_points")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.publisher = self.create_publisher(LaserScan, output_topic, 10)
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.callback, 10
        )
        self.get_logger().info(f"Creating placeholder scans from {input_topic} -> {output_topic}")

    def callback(self, msg: PointCloud2):
        # Placeholder: publishes an empty scan with header copied.
        scan = LaserScan()
        scan.header = msg.header
        scan.angle_min = 0.0
        scan.angle_max = 0.0
        scan.angle_increment = 0.0
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = 0.0
        scan.range_max = 0.0
        scan.ranges = [math.inf]
        scan.intensities = [0.0]
        self.publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToLaserScanNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
