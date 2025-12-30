#!/usr/bin/env python3
"""
CostmapSafetyLayer
------------------
Simple safety layer for SHOBOT AMR.

Publishes /safety_stop (Bool) when:
  - Any LaserScan range is closer than a threshold, OR
  - Any detected landmark (PoseArray) is within a stop radius.
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class CostmapSafetyLayer(Node):
    """Safety stop generator for LiDAR + landmark-based protection."""

    def __init__(self):
        super().__init__("shobot_costmap_safety_layer")

        # ---------------- Parameters ----------------
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("safety_stop_topic", "/safety_stop")
        self.declare_parameter("scan_threshold", 0.4)
        self.declare_parameter("landmark_topic", "")
        self.declare_parameter("landmark_stop_radius", 0.5)

        scan_topic = self.get_parameter("scan_topic").value
        self.stop_topic = self.get_parameter("safety_stop_topic").value
        self.scan_threshold = float(self.get_parameter("scan_threshold").value)

        self.landmark_topic = self.get_parameter("landmark_topic").value
        self.landmark_stop_radius = float(
            self.get_parameter("landmark_stop_radius").value
        )

        # ---------------- Publisher ----------------
        self.stop_pub = self.create_publisher(Bool, self.stop_topic, 10)

        # ---------------- Subscriptions ----------------
        self.create_subscription(LaserScan, scan_topic, self.scan_cb, 10)

        if self.landmark_topic:
            self.create_subscription(
                PoseArray, self.landmark_topic, self.landmark_cb, 10
            )
            self.get_logger().info(
                f"Safety layer active | scan: '{scan_topic}', "
                f"landmarks: '{self.landmark_topic}', "
                f"stop topic: '{self.stop_topic}'"
            )
        else:
            self.get_logger().info(
                f"Safety layer active | scan: '{scan_topic}', "
                f"stop topic: '{self.stop_topic}'"
            )

        # ---------------- Internal State ----------------
        self.landmark_stop: bool = False
        self.latest_scan_stop: bool = False

    # ------------------------------------------------------------------
    # LaserScan Callback
    # ------------------------------------------------------------------
    def scan_cb(self, msg: LaserScan):
        """Determine safety stop based on LaserScan ranges."""

        min_range: Optional[float] = None

        for r in msg.ranges:
            if math.isfinite(r):
                if min_range is None or r < min_range:
                    min_range = r

        # Conservative behavior: if scan is invalid, STOP
        if min_range is None:
            self.get_logger().warn("No valid LaserScan ranges received → STOP")
            self.latest_scan_stop = True
        else:
            self.latest_scan_stop = min_range < self.scan_threshold

        stop = self.latest_scan_stop or self.landmark_stop
        self.stop_pub.publish(Bool(data=stop))

    # ------------------------------------------------------------------
    # Landmark Callback
    # ------------------------------------------------------------------
    def landmark_cb(self, msg: PoseArray):
        """Trigger stop if any landmark is within stop radius."""
        self.landmark_stop = False

        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y

            if math.hypot(x, y) < self.landmark_stop_radius:
                self.landmark_stop = True
                break


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = CostmapSafetyLayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
