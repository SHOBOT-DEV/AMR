#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class PlaceholderNode(Node):
    def __init__(self):
        super().__init__("shobot_trajectory_controller")
        self.get_logger().info("shobot_trajectory_controller placeholder node started")


def main(args=None):
    rclpy.init(args=args)
    node = PlaceholderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
