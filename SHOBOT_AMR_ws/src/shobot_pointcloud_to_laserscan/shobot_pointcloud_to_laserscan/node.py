#!/usr/bin/env python3
"""
PointCloud2 → LaserScan
=========================================================
Projects a 3D PointCloud2 into a 2D planar LaserScan.
Optimized for AMR costmaps and Nav2 obstacle detection.
"""

import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan, PointCloud2


class PointCloudToLaserScanNode(Node):
    """Convert 3D point cloud into 2D LaserScan."""

    def __init__(self):
        super().__init__("shobot_pointcloud_to_laserscan")

        # ---------------- Parameters ----------------
        self.declare_parameter("input_topic", "/points")
        self.declare_parameter("output_topic", "/scan_from_points")
        self.declare_parameter("scan_frame", "base_scan")

        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)
        self.declare_parameter("angle_increment", math.radians(1.0))

        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 20.0)

        self.declare_parameter("min_height", -0.5)
        self.declare_parameter("max_height", 1.0)

        # ---------------- Load Parameters ----------------
        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.scan_frame = self.get_parameter("scan_frame").value

        self.angle_min = float(self.get_parameter("angle_min").value)
        self.angle_max = float(self.get_parameter("angle_max").value)
        self.angle_increment = float(self.get_parameter("angle_increment").value)

        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)

        self.min_height = float(self.get_parameter("min_height").value)
        self.max_height = float(self.get_parameter("max_height").value)

        if self.angle_increment <= 0.0:
            raise ValueError("angle_increment must be > 0")

        self.num_readings = max(
            1,
            int((self.angle_max - self.angle_min) / self.angle_increment) + 1,
        )

        # ---------------- QoS (sensor data) ----------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---------------- Publisher / Subscriber ----------------
        self.publisher = self.create_publisher(
            LaserScan, self.output_topic, sensor_qos
        )
        self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.callback,
            sensor_qos,
        )

        self.get_logger().info(
            f"PointCloud → LaserScan: {self.input_topic} → {self.output_topic} "
            f"(frame={self.scan_frame}, bins={self.num_readings})"
        )

    # ------------------------------------------------------------------
    def callback(self, cloud: PointCloud2):
        """Convert incoming PointCloud2 into planar LaserScan."""

        ranges: List[float] = [math.inf] * self.num_readings

        amin = self.angle_min
        amax = self.angle_max
        aincr = self.angle_increment
        zmin = self.min_height
        zmax = self.max_height
        rmin = self.range_min
        rmax = self.range_max

        for (x, y, z) in pc2.read_points(
            cloud, field_names=("x", "y", "z"), skip_nans=True
        ):
            # Height filter
            if z < zmin or z > zmax:
                continue

            r = math.hypot(x, y)
            if r < rmin or r > rmax:
                continue

            angle = math.atan2(y, x)
            if angle < amin or angle > amax:
                continue

            idx = int((angle - amin) / aincr)
            if 0 <= idx < self.num_readings and r < ranges[idx]:
                ranges[idx] = r

        # ---------------- Build LaserScan ----------------
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.scan_frame

        scan.angle_min = amin
        scan.angle_max = amax
        scan.angle_increment = aincr

        scan.range_min = rmin
        scan.range_max = rmax

        scan.time_increment = 0.0
        scan.scan_time = 0.0

        scan.ranges = ranges
        scan.intensities = [float("nan")] * self.num_readings

        self.publisher.publish(scan)


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToLaserScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
