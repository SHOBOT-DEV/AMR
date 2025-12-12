#!/usr/bin/env python3
"""
Simple safety layer for SHOBOT AMR.
Publishes /safety_stop whenever:
  - LaserScan shows any obstacle closer than threshold, OR
  - A detected landmark (PoseArray) is within landmark_stop_radius.
"""
import math

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
        self.declare_parameter("threshold", 0.4)
        self.declare_parameter("landmark_topic", "")
        self.declare_parameter("landmark_stop_radius", 0.5)

        scan_topic = self.get_parameter("scan_topic").value
        self.stop_topic = self.get_parameter("safety_stop_topic").value
        self.threshold = float(self.get_parameter("threshold").value)

        self.landmark_topic = self.get_parameter("landmark_topic").value
        self.landmark_stop_radius = float(self.get_parameter("landmark_stop_radius").value)

        # ---------------- Publisher ----------------
        self.stop_pub = self.create_publisher(Bool, self.stop_topic, 10)

        # ---------------- Subscriptions ----------------
        self.create_subscription(LaserScan, scan_topic, self.scan_cb, 10)

        if self.landmark_topic:
            self.create_subscription(PoseArray, self.landmark_topic, self.landmark_cb, 10)
            self.get_logger().info(
                f"Safety layer active: scanning '{scan_topic}' + landmarks '{self.landmark_topic}', "
                f"publishing -> {self.stop_topic}"
            )
        else:
            self.get_logger().info(
                f"Safety layer active: scanning '{scan_topic}', publishing -> {self.stop_topic}"
            )

        # State variable for landmark stop
        self.landmark_stop = False

    # ----------------------------------------------------------------------
    # LaserScan Callback
    # ----------------------------------------------------------------------
    def scan_cb(self, msg: LaserScan):
        """Check minimum valid scan range to determine if safety stop is required."""

        # Filter out invalid values
        valid_ranges = [r for r in msg.ranges if not math.isnan(r) and not math.isinf(r)]

        if valid_ranges:
            min_range = min(valid_ranges)
        else:
            # If no valid ranges, assume safe — but still check landmarks
            min_range = math.inf

        obstacle_stop = min_range < self.threshold
        stop = obstacle_stop or self.landmark_stop

        self.stop_pub.publish(Bool(data=stop))

    # ----------------------------------------------------------------------
    # Landmark Callback
    # ----------------------------------------------------------------------
    def landmark_cb(self, msg: PoseArray):
        """Stop robot when any landmark is within landmark_stop_radius."""
        self.landmark_stop = False

        for pose in msg.poses:
            x, y = pose.position.x, pose.position.y

            # Compute distance
            dist = math.hypot(x, y)
            if dist < self.landmark_stop_radius:
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
