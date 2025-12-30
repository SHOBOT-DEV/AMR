#!/usr/bin/env python3
"""
Assemble multiple PointCloud2 streams into a single combined cloud.

- Ensures field compatibility
- Uses sensor-friendly QoS
- Optional frame override
- Optional source timestamp preservation
"""

from typing import List, Dict, Optional

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
        self.declare_parameter("frame_id", "")  # optional override
        self.declare_parameter("keep_source_timestamps", False)

        self.input_topics: List[str] = list(
            self.get_parameter("input_topics").value
        )
        self.output_topic: str = self.get_parameter("output_topic").value
        self.frame_override: str = self.get_parameter("frame_id").value
        self.keep_source_ts: bool = bool(
            self.get_parameter("keep_source_timestamps").value
        )

        # ---------------- QoS ----------------
        cloud_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        # ---------------- Storage ----------------
        self.buffers: Dict[str, Optional[PointCloud2]] = {
            topic: None for topic in self.input_topics
        }

        # ---------------- Publisher ----------------
        self.publisher = self.create_publisher(
            PointCloud2, self.output_topic, cloud_qos
        )

        # ---------------- Subscriptions ----------------
        for topic in self.input_topics:
            self.create_subscription(
                PointCloud2,
                topic,
                self._make_callback(topic),
                cloud_qos,
            )

        self.get_logger().info(
            f"PointCloudAssembler active:\n"
            f"  Inputs: {self.input_topics}\n"
            f"  Output: {self.output_topic}\n"
            f"  Frame override: {self.frame_override or 'inherit'}"
        )

    # ------------------------------------------------------------------
    def _make_callback(self, topic: str):
        """Create a topic-safe callback (avoids lambda late-binding)."""

        def callback(msg: PointCloud2):
            self.store_cloud(msg, topic)

        return callback

    # ------------------------------------------------------------------
    def store_cloud(self, msg: PointCloud2, topic: str):
        """Store the latest cloud from a given topic."""
        self.buffers[topic] = msg

        # Publish only when all clouds are available
        if all(cloud is not None for cloud in self.buffers.values()):
            self.publish_combined()

    # ------------------------------------------------------------------
    def publish_combined(self):
        """Merge all stored clouds into one PointCloud2."""

        clouds = list(self.buffers.values())
        assert clouds and clouds[0] is not None

        reference = clouds[0]

        # ---------------- Field consistency check ----------------
        for cloud in clouds[1:]:
            if not self._compare_fields(reference.fields, cloud.fields):
                self.get_logger().error(
                    "PointField mismatch detected — cannot assemble clouds."
                )
                return

        # ---------------- Header ----------------
        header = Header()
        header.stamp = (
            reference.header.stamp
            if self.keep_source_ts
            else self.get_clock().now().to_msg()
        )
        header.frame_id = self.frame_override or reference.header.frame_id

        # ---------------- Merge points ----------------
        field_names = [f.name for f in reference.fields]
        combined_points = []

        for cloud in clouds:
            combined_points.extend(
                pc2.read_points(cloud, field_names=field_names, skip_nans=True)
            )

        combined_cloud = pc2.create_cloud(
            header, reference.fields, combined_points
        )

        self.publisher.publish(combined_cloud)

    # ------------------------------------------------------------------
    @staticmethod
    def _compare_fields(
        f1: List[PointField], f2: List[PointField]
    ) -> bool:
        """Ensure PointField layouts are identical."""
        if len(f1) != len(f2):
            return False
        for a, b in zip(f1, f2):
            if (
                a.name != b.name
                or a.datatype != b.datatype
                or a.count != b.count
                or a.offset != b.offset
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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
