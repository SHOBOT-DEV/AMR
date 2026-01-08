#!/usr/bin/env python3
"""
Lightweight PointCloud2 pass-through filter for SHOBOT AMR.
Includes QoS, frame override, and optional height/box filtering.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


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
            depth=1
        )

        self.publisher = self.create_publisher(PointCloud2, output_topic, sensor_qos)
        self.create_subscription(PointCloud2, input_topic, self.callback, sensor_qos)

        self.get_logger().info(
            f"PointCloudFilter active: {input_topic} → {output_topic}\n"
            f"  Height filter: {self.enable_height}  (min_z={self.min_z}, max_z={self.max_z})\n"
            f"  Frame override: '{self.frame_id or 'inherit'}'"
        )

    # ----------------------------------------------------------------------
    def callback(self, msg: PointCloud2):
        """Filter and republish pointcloud."""

        # ---------------- Optional height filtering ----------------
        if self.enable_height:
            points = []
            min_z = self.min_z
            max_z = self.max_z

            for p in pc2.read_points(msg, skip_nans=True):
                x, y, z = p[:3]
                if min_z <= z <= max_z:
                    points.append(p)

            msg = pc2.create_cloud(msg.header, msg.fields, points)

        # ---------------- Frame override if enabled ----------------
        if self.frame_id:
            msg.header.frame_id = self.frame_id

        # ---------------- Timestamp fix if missing ----------------
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            msg.header.stamp = self.get_clock().now().to_msg()

        # ---------------- Publish final cloud ----------------
        self.publisher.publish(msg)


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
