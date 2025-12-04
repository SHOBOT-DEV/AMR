#!/usr/bin/env python3
import math
from typing import Dict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanMergerNode(Node):
    """Merge multiple LaserScan topics by taking the minimum range per index."""

    def __init__(self):
        super().__init__("shobot_scan_merger")
        self.declare_parameter("input_topics", ["/scan_front", "/scan_rear"])
        self.declare_parameter("output_topic", "/scan_merged")

        input_topics = self.get_parameter("input_topics").value
        output_topic = self.get_parameter("output_topic").value

        self.buffers: Dict[str, LaserScan | None] = {t: None for t in input_topics}
        self.expected_len = None
        self.publisher = self.create_publisher(LaserScan, output_topic, 10)

        for topic in input_topics:
            self.create_subscription(LaserScan, topic, self._make_cb(topic), 10)

        self.get_logger().info(f"Merging scans {input_topics} -> {output_topic}")

    def _make_cb(self, topic_name):
        def cb(msg: LaserScan):
            if self.expected_len is None:
                self.expected_len = len(msg.ranges)
            self.buffers[topic_name] = msg
            self.try_publish()

        return cb

    def try_publish(self):
        if any(v is None for v in self.buffers.values()):
            return
        sample = next(iter(self.buffers.values()))
        merged = LaserScan()
        merged.header = sample.header
        merged.angle_min = sample.angle_min
        merged.angle_max = sample.angle_max
        merged.angle_increment = sample.angle_increment
        merged.time_increment = sample.time_increment
        merged.scan_time = sample.scan_time
        merged.range_min = min(v.range_min for v in self.buffers.values())
        merged.range_max = max(v.range_max for v in self.buffers.values())

        ranges = []
        intensities = []
        for i in range(self.expected_len):
            vals = []
            ints = []
            for msg in self.buffers.values():
                if len(msg.ranges) == self.expected_len:
                    vals.append(msg.ranges[i])
                    if msg.intensities:
                        ints.append(msg.intensities[i])
            vals = [v for v in vals if not math.isinf(v)]
            ranges.append(min(vals) if vals else math.inf)
            intensities.append(max(ints) if ints else 0.0)
        merged.ranges = ranges
        merged.intensities = intensities
        self.publisher.publish(merged)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMergerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
