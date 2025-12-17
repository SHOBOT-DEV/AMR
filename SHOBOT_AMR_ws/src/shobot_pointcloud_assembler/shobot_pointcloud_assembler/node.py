#!/usr/bin/env python3
"""Assemble multiple PointCloud2 streams into a single combined cloud."""
from typing import List

import rclpy
import sensor_msgs.point_cloud2 as pc2
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class PointCloudAssemblerNode(Node):
    """Concatenate multiple PointCloud2 inputs into one output cloud."""

    def __init__(self):
        super().__init__("shobot_pointcloud_assembler")
        self.declare_parameter("input_topics", ["/points_filtered"])
        self.declare_parameter("output_topic", "/points_assembled")
        self.declare_parameter("queue_size", 2)

        self.output_topic = self.get_parameter("output_topic").value
        input_topics = self.get_parameter("input_topics").value
        self.queue_size = int(self.get_parameter("queue_size").value)

        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.buffers = {topic: None for topic in input_topics}

        for topic in input_topics:
            self.create_subscription(PointCloud2, topic, lambda msg, t=topic: self.store_cloud(msg, t), 10)

        self.get_logger().info(f"Assembling clouds from {input_topics} -> {self.output_topic}")

    def store_cloud(self, msg: PointCloud2, topic: str):
        self.buffers[topic] = msg
        if all(self.buffers.values()):
            self.publish_combined()

    def publish_combined(self):
        clouds = list(self.buffers.values())
        header = Header()
        header.stamp = clouds[0].header.stamp
        header.frame_id = clouds[0].header.frame_id

        combined_points = []
        for cloud in clouds:
            combined_points.extend(pc2.read_points(cloud, field_names=[f.name for f in cloud.fields], skip_nans=True))

        # Use fields from first cloud
        fields: List[PointField] = clouds[0].fields
        combined = pc2.create_cloud(header, fields, combined_points)
        self.publisher.publish(combined)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudAssemblerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
