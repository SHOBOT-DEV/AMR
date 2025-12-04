#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class PointCloudFilterNode(Node):
    """Pass-through point cloud filter with configurable topics."""

    def __init__(self):
        super().__init__("shobot_pointcloud_filter")
        self.declare_parameter("input_topic", "/points_raw")
        self.declare_parameter("output_topic", "/points_filtered")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.publisher = self.create_publisher(PointCloud2, output_topic, 10)
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.callback, 10
        )
        self.get_logger().info(f"Forwarding {input_topic} -> {output_topic}")

    def callback(self, msg: PointCloud2):
        # Forward unchanged; extend with filtering pipeline as needed.
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
