#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LaserFilterNode(Node):
    """Simple pass-through laser filter with configurable input/output topics."""

    def __init__(self):
        super().__init__("shobot_laser_filters")
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_filtered")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.publisher = self.create_publisher(LaserScan, output_topic, 10)
        self.subscription = self.create_subscription(
            LaserScan, input_topic, self.scan_callback, 10
        )
        self.get_logger().info(f"Forwarding {input_topic} -> {output_topic}")

    def scan_callback(self, msg: LaserScan):
        # Pass-through for now; extend with filtering logic.
        self.publisher.publish(msg)


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
