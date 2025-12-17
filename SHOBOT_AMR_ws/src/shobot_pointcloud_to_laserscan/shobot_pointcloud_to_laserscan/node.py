#!/usr/bin/env python3
"""Project 3D PointCloud2 data onto a planar LaserScan for 2D costmaps."""
import math
from typing import List

import rclpy
import sensor_msgs.point_cloud2 as pc2
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2


class PointCloudToLaserScanNode(Node):
    """Convert PointCloud2 to LaserScan with height filtering and angular bins."""

    def __init__(self):
        super().__init__("shobot_pointcloud_to_laserscan")
        self.declare_parameter("input_topic", "/points")
        self.declare_parameter("output_topic", "/scan_from_points")
        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)
        self.declare_parameter("angle_increment", math.radians(1.0))
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 20.0)
        self.declare_parameter("min_height", -0.5)
        self.declare_parameter("max_height", 1.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.angle_min = float(self.get_parameter("angle_min").value)
        self.angle_max = float(self.get_parameter("angle_max").value)
        self.angle_increment = float(self.get_parameter("angle_increment").value)
        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.min_height = float(self.get_parameter("min_height").value)
        self.max_height = float(self.get_parameter("max_height").value)

        self.publisher = self.create_publisher(LaserScan, output_topic, 10)
        self.create_subscription(PointCloud2, input_topic, self.callback, 10)
        self.get_logger().info(
            f"Converting PointCloud2 from {input_topic} to LaserScan on {output_topic}"
        )

    def callback(self, msg: PointCloud2):
        num_readings = int((self.angle_max - self.angle_min) / self.angle_increment) + 1
        ranges: List[float] = [math.inf] * num_readings

        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            if z < self.min_height or z > self.max_height:
                continue
            r = math.hypot(x, y)
            if r < self.range_min or r > self.range_max:
                continue
            angle = math.atan2(y, x)
            if angle < self.angle_min or angle > self.angle_max:
                continue
            idx = int((angle - self.angle_min) / self.angle_increment)
            if 0 <= idx < num_readings and r < ranges[idx]:
                ranges[idx] = r

        scan = LaserScan()
        scan.header = msg.header
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        scan.intensities = [0.0] * num_readings
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
