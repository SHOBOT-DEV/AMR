#!/usr/bin/env python3
"""
SHOBOT - LaserScan Merger
=========================================
Merges multiple LaserScan inputs into a single 360° or enhanced scan.
Takes minimum range per angle and handles missing/inf values properly.
"""

import math
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanMergerNode(Node):
    """Merge multiple LaserScan topics by taking the minimum range per index."""

    def __init__(self):
        super().__init__("shobot_scan_merger")

        # ---------------- Parameters ----------------
        self.declare_parameter("input_topics", ["/scan_front", "/scan_rear"])
        self.declare_parameter("output_topic", "/scan_merged")
        self.declare_parameter("frame_id", "base_laser")
        self.declare_parameter("publish_rate_hz", 10.0)

        self.input_topics = self.get_parameter("input_topics").value
        self.output_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.rate = float(self.get_parameter("publish_rate_hz").value)

        # Storage for latest scans
        self.buffers: Dict[str, Optional[LaserScan]] = {
            t: None for t in self.input_topics
        }
        self.expected_len = None

        # Publisher
        self.pub = self.create_publisher(LaserScan, self.output_topic, 10)

        # Subscriptions
        for topic in self.input_topics:
            self.create_subscription(LaserScan, topic, self._cb(topic), 10)

        # Timer-based publishing (ensures stable rate)
        self.create_timer(1.0 / self.rate, self.try_publish)

        self.get_logger().info(
            f"Merging scans {self.input_topics} → {self.output_topic} at {self.rate} Hz"
        )

    # ----------------------------------------------------------------------
    def _cb(self, topic):
        def callback(msg: LaserScan):
            if self.expected_len is None:
                self.expected_len = len(msg.ranges)

            if len(msg.ranges) != self.expected_len:
                self.get_logger().warn(
                    f"Scan length mismatch on {topic}. Expected {self.expected_len}, got {len(msg.ranges)}"
                )
                return

            self.buffers[topic] = msg

        return callback

    # ----------------------------------------------------------------------
    def try_publish(self):
        """Publish merged scan at fixed rate."""
        if any(v is None for v in self.buffers.values()):
            return

        sample = next(iter(self.buffers.values()))
        now = self.get_clock().now().to_msg()

        merged = LaserScan()
        merged.header.stamp = now
        merged.header.frame_id = self.frame_id

        # Copy common scan parameters
        merged.angle_min = sample.angle_min
        merged.angle_max = sample.angle_max
        merged.angle_increment = sample.angle_increment
        merged.time_increment = sample.time_increment
        merged.scan_time = sample.scan_time

        # Conservative range limits
        merged.range_min = min(msg.range_min for msg in self.buffers.values())
        merged.range_max = max(msg.range_max for msg in self.buffers.values())

        merged_ranges = []
        merged_intensities = []

        for i in range(self.expected_len):
            # Gather ith range across all scans
            values = []
            intens = []

            for msg in self.buffers.values():
                r = msg.ranges[i]

                # Keep finite values only
                if not math.isinf(r):
                    values.append(r)

                if msg.intensities:
                    intens.append(msg.intensities[i])

            # Minimum distance OR inf (no detection)
            merged_ranges.append(min(values) if values else math.inf)

            # Choose max intensity or nan
            merged_intensities.append(max(intens) if intens else float("nan"))

        merged.ranges = merged_ranges
        merged.intensities = merged_intensities

        self.pub.publish(merged)

    # ----------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = ScanMergerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
