#!/usr/bin/env python3
"""
SHOBOT - LaserScan Merger
=========================================================
Merges multiple LaserScan inputs into a single scan by
taking the minimum valid range per angle index.

Requirements:
- All input scans must have identical angular configuration
  (angle_min, angle_max, angle_increment).
"""

import math
from typing import Dict, Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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

        self.input_topics: List[str] = list(
            self.get_parameter("input_topics").value
        )
        self.output_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.rate_hz = max(
            float(self.get_parameter("publish_rate_hz").value), 0.1
        )

        # Latest scans
        self.buffers: Dict[str, Optional[LaserScan]] = {
            t: None for t in self.input_topics
        }

        self.expected_len: Optional[int] = None
        self.ref_angle_min: Optional[float] = None
        self.ref_angle_inc: Optional[float] = None

        # ---------------- Publisher ----------------
        self.pub = self.create_publisher(LaserScan, self.output_topic, 10)

        # ---------------- Subscriptions ----------------
        for topic in self.input_topics:
            self.create_subscription(
                LaserScan,
                topic,
                self._make_cb(topic),
                qos_profile_sensor_data,
            )

        # ---------------- Timer ----------------
        self.create_timer(1.0 / self.rate_hz, self.publish_merged)

        self.get_logger().info(
            f"LaserScan merger started\n"
            f"Inputs : {self.input_topics}\n"
            f"Output : {self.output_topic}\n"
            f"Rate   : {self.rate_hz} Hz"
        )

    # ------------------------------------------------------------------
    def _make_cb(self, topic: str):
        def callback(msg: LaserScan):
            # Initialize reference geometry
            if self.expected_len is None:
                self.expected_len = len(msg.ranges)
                self.ref_angle_min = msg.angle_min
                self.ref_angle_inc = msg.angle_increment

            # Validate geometry
            if (
                len(msg.ranges) != self.expected_len
                or msg.angle_min != self.ref_angle_min
                or msg.angle_increment != self.ref_angle_inc
            ):
                self.get_logger().warn(
                    f"Scan geometry mismatch on {topic}; ignoring scan"
                )
                return

            self.buffers[topic] = msg

        return callback

    # ------------------------------------------------------------------
    def publish_merged(self):
        """Publish merged scan at a fixed rate."""
        if any(scan is None for scan in self.buffers.values()):
            return

        scans = [self.buffers[t] for t in self.input_topics]
        sample = scans[0]

        merged = LaserScan()
        merged.header.stamp = self.get_clock().now().to_msg()
        merged.header.frame_id = self.frame_id

        # Copy scan geometry
        merged.angle_min = sample.angle_min
        merged.angle_max = sample.angle_max
        merged.angle_increment = sample.angle_increment
        merged.time_increment = sample.time_increment
        merged.scan_time = sample.scan_time
        merged.range_min = min(s.range_min for s in scans)
        merged.range_max = max(s.range_max for s in scans)

        ranges = []
        intensities = []

        for i in range(self.expected_len):
            valid_ranges = []
            valid_intensities = []

            for scan in scans:
                r = scan.ranges[i]
                if not math.isinf(r) and not math.isnan(r):
                    valid_ranges.append(r)

                if scan.intensities and i < len(scan.intensities):
                    valid_intensities.append(scan.intensities[i])

            ranges.append(min(valid_ranges) if valid_ranges else math.inf)
            intensities.append(
                max(valid_intensities) if valid_intensities else float("nan")
            )

        merged.ranges = ranges
        merged.intensities = intensities

        self.pub.publish(merged)

    # ------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = ScanMergerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
