#!/usr/bin/env python3
"""Simple safety layer: publish stop flag when obstacles too close."""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class CostmapSafetyLayer(Node):
    """Checks LaserScan ranges against a threshold and emits /safety_stop."""

    def __init__(self):
        super().__init__("shobot_costmap_safety_layer")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("safety_stop_topic", "/safety_stop")
        self.declare_parameter("threshold", 0.4)

        scan_topic = self.get_parameter("scan_topic").value
        self.stop_topic = self.get_parameter("safety_stop_topic").value
        self.threshold = float(self.get_parameter("threshold").value)

        self.stop_pub = self.create_publisher(Bool, self.stop_topic, 10)
        self.create_subscription(LaserScan, scan_topic, self.scan_cb, 10)
        self.get_logger().info(f"Safety layer monitoring {scan_topic}, publishing {self.stop_topic}")

    def scan_cb(self, msg: LaserScan):
        min_range = min(msg.ranges) if msg.ranges else math.inf
        stop = min_range < self.threshold
        self.stop_pub.publish(Bool(data=stop))


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
