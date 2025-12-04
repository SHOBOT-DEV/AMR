#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class PointCloudAssemblerNode(Node):
    """Pass-through assembler that forwards the first input topic to the output."""

    def __init__(self):
        super().__init__("shobot_pointcloud_assembler")
        self.declare_parameter("input_topics", ["/points_filtered"])
        self.declare_parameter("output_topic", "/points_assembled")

        self.output_topic = self.get_parameter("output_topic").value
        input_topics = self.get_parameter("input_topics").value

        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.subs = []
        for topic in input_topics:
            self.subs.append(
                self.create_subscription(PointCloud2, topic, self.callback, 10)
            )
        self.get_logger().info(f"Assembling first available from {input_topics} -> {self.output_topic}")

    def callback(self, msg: PointCloud2):
        # Forward first received input cloud; extend to real fusion if needed.
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudAssemblerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
