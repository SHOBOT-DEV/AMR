#!/usr/bin/env python3
"""Laser filter supporting range clipping and NaN replacement."""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LaserFilterNode(Node):
    """Filter LaserScan ranges: clamp min/max, replace inf/nan, and republish."""

    def __init__(self):
        super().__init__("shobot_laser_filters")
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_filtered")
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 10.0)
        self.declare_parameter("replace_invalid_with", 0.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.replace_invalid_with = float(self.get_parameter("replace_invalid_with").value)

        self.publisher = self.create_publisher(LaserScan, output_topic, 10)
        self.create_subscription(LaserScan, input_topic, self.scan_callback, 10)
        self.get_logger().info(
            f"Filtering LaserScan {input_topic} -> {output_topic} (clamp {self.range_min}-{self.range_max} m)"
        )

    def scan_callback(self, msg: LaserScan):
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = self.range_min
        filtered.range_max = self.range_max

        ranges = []
        for r in msg.ranges:
            if math.isnan(r) or math.isinf(r):
                ranges.append(self.replace_invalid_with)
            else:
                ranges.append(max(self.range_min, min(self.range_max, r)))
        filtered.ranges = ranges
        filtered.intensities = msg.intensities

        self.publisher.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = LaserFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
