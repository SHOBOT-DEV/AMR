#!/usr/bin/env python3
"""
Assemble multiple PointCloud2 streams into a single combined cloud.
Includes QoS, safety checks, and field consistency verification.
"""

from typing import List, Dict

import rclpy
import sensor_msgs.point_cloud2 as pc2
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class PointCloudAssemblerNode(Node):
    """Combine multiple PointCloud2 clouds into one output cloud."""

    def __init__(self):
        super().__init__("shobot_pointcloud_assembler")

        # ---------------- Parameters ----------------
        self.declare_parameter("input_topics", ["/points_filtered"])
        self.declare_parameter("output_topic", "/points_assembled")
        self.declare_parameter("frame_id", "")  # frame override optional
        self.declare_parameter("keep_source_timestamps", False)

        input_topics = self.get_parameter("input_topics").value
        self.output_topic = self.get_parameter("output_topic").value
        self.frame_override = self.get_parameter("frame_id").value
        self.keep_source_ts = bool(self.get_parameter("keep_source_timestamps").value)

        # ---------------- QoS ----------------
        cloud_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        # ---------------- Storage ----------------
        self.buffers: Dict[str, PointCloud2 | None] = {topic: None for topic in input_topics}

        self.publisher = self.create_publisher(PointCloud2, self.output_topic, cloud_qos)

        # ---------------- Subscriptions ----------------
        for topic in input_topics:
            self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, t=topic: self.store_cloud(msg, t),
                cloud_qos,
            )

        self.get_logger().info(
            f"Assembling clouds from {input_topics} → {self.output_topic}\n"
            f"Frame override: {self.frame_override or 'inherit'}"
        )

    # ------------------------------------------------------------------
    def store_cloud(self, msg: PointCloud2, topic: str):
        """Store latest cloud from each topic."""
        self.buffers[topic] = msg

        # When all have arrived at least once → publish
        if all(self.buffers.values()):
            self.publish_combined()

    # ------------------------------------------------------------------
    def publish_combined(self):
        """Merge all stored clouds safely."""

        clouds = list(self.buffers.values())
        first = clouds[0]

        # ---------------- Validate field structure consistency ----------------
        if not all(self._compare_fields(first.fields, c.fields) for c in clouds):
            self.get_logger().error("Field mismatch detected; cannot combine clouds!")
            return

        # ---------------- Build header ----------------
        header = Header()
        header.stamp = (
            first.header.stamp if self.keep_source_ts else self.get_clock().now().to_msg()
        )
        header.frame_id = self.frame_override or first.header.frame_id

        # ---------------- Merge points ----------------
        fields = first.fields
        combined_points = []

        for cloud in clouds:
            pts = pc2.read_points(cloud, field_names=[f.name for f in fields], skip_nans=True)
            combined_points.extend(pts)

        combined_cloud = pc2.create_cloud(header, fields, combined_points)
        self.publisher.publish(combined_cloud)

    # ------------------------------------------------------------------
    @staticmethod
    def _compare_fields(fields1: List[PointField], fields2: List[PointField]) -> bool:
        """Ensure point fields match before merging."""
        if len(fields1) != len(fields2):
            return False
        for f1, f2 in zip(fields1, fields2):
            if (
                f1.name != f2.name
                or f1.datatype != f2.datatype
                or f1.count != f2.count
                or f1.offset != f2.offset
            ):
                return False
        return True


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PointCloudAssemblerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
