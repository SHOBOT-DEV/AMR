#!/usr/bin/env python3
"""
Lightweight PointCloud2 pass-through filter for SHOBOT AMR.
Includes QoS, frame override, and optional height filtering.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


class PointCloudFilterNode(Node):
    """Simple point cloud filter & forwarder with optional height filtering."""

    def __init__(self):
        super().__init__("shobot_pointcloud_filter")

        # ---------------- Parameters ----------------
        self.declare_parameter("input_topic", "/points_raw")
        self.declare_parameter("output_topic", "/points_filtered")

        # Optional filter parameters
        self.declare_parameter("frame_id", "")  # override frame_id if provided
        self.declare_parameter("min_z", -999.0)
        self.declare_parameter("max_z", 999.0)
        self.declare_parameter("enable_height_filter", False)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.frame_id = self.get_parameter("frame_id").value
        self.min_z = float(self.get_parameter("min_z").value)
        self.max_z = float(self.get_parameter("max_z").value)
        self.enable_height = bool(self.get_parameter("enable_height_filter").value)

        # ---------------- QoS for high-rate sensors ----------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.publisher = self.create_publisher(PointCloud2, output_topic, sensor_qos)
        self.create_subscription(PointCloud2, input_topic, self.callback, sensor_qos)

        self.get_logger().info(
            f"PointCloudFilter active:\n"
            f"  {input_topic} → {output_topic}\n"
            f"  Height filter: {self.enable_height} (min_z={self.min_z}, max_z={self.max_z})\n"
            f"  Frame override: '{self.frame_id or 'inherit'}'"
        )

    # ------------------------------------------------------------------
    def callback(self, msg: PointCloud2):
        """Filter and republish PointCloud2."""

        header = msg.header
        fields = msg.fields

        # ---------------- Optional height filtering ----------------
        if self.enable_height:
            filtered_points = []

            for x, y, z in pc2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True,
            ):
                if self.min_z <= z <= self.max_z:
                    filtered_points.append((x, y, z))

            out_cloud = pc2.create_cloud(header, fields, filtered_points)
        else:
            # Pass-through (reuse data safely)
            out_cloud = PointCloud2()
            out_cloud.header = header
            out_cloud.height = msg.height
            out_cloud.width = msg.width
            out_cloud.fields = msg.fields
            out_cloud.is_bigendian = msg.is_bigendian
            out_cloud.point_step = msg.point_step
            out_cloud.row_step = msg.row_step
            out_cloud.data = msg.data
            out_cloud.is_dense = msg.is_dense

        # ---------------- Frame override ----------------
        if self.frame_id:
            out_cloud.header.frame_id = self.frame_id

        # ---------------- Timestamp safety ----------------
        if (
            out_cloud.header.stamp.sec == 0
            and out_cloud.header.stamp.nanosec == 0
        ):
            out_cloud.header.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(out_cloud)


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
