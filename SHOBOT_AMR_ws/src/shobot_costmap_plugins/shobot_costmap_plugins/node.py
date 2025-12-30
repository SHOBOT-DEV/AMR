#!/usr/bin/env python3
"""
CostmapBuilder (ROS 2)
---------------------
Projects PointCloud2 obstacle data into a 2D OccupancyGrid
with height filtering, TF2 transform, and inflation.

Nav2-compatible and AMR-safe.
"""

from math import floor
from typing import List, Tuple, Set

import numpy as np
import rclpy
import sensor_msgs.point_cloud2 as pc2

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped

import tf2_ros
from tf2_ros import TransformException


class CostmapBuilder(Node):
    """Convert PointCloud2 obstacles into a 2D OccupancyGrid."""

    def __init__(self):
        super().__init__("shobot_costmap_builder")

        # -------------------- Parameters --------------------
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("resolution", 0.1)
        self.declare_parameter("width", 20.0)
        self.declare_parameter("height", 20.0)
        self.declare_parameter("origin_x", -10.0)
        self.declare_parameter("origin_y", -10.0)
        self.declare_parameter("obstacle_topic", "/obstacles")
        self.declare_parameter("costmap_topic", "/shobot_costmap")
        self.declare_parameter("min_obstacle_height", 0.05)
        self.declare_parameter("max_obstacle_height", 2.0)
        self.declare_parameter("inflation_radius", 0.3)

        # -------------------- Read Parameters --------------------
        self.frame_id = self.get_parameter("frame_id").value
        self.resolution = float(self.get_parameter("resolution").value)
        self.width_m = float(self.get_parameter("width").value)
        self.height_m = float(self.get_parameter("height").value)
        self.origin_x = float(self.get_parameter("origin_x").value)
        self.origin_y = float(self.get_parameter("origin_y").value)
        self.min_h = float(self.get_parameter("min_obstacle_height").value)
        self.max_h = float(self.get_parameter("max_obstacle_height").value)
        self.inflation_radius = float(self.get_parameter("inflation_radius").value)

        obstacle_topic = self.get_parameter("obstacle_topic").value
        self.costmap_topic = self.get_parameter("costmap_topic").value

        # -------------------- Derived --------------------
        self.width_cells = int(self.width_m / self.resolution)
        self.height_cells = int(self.height_m / self.resolution)
        self.inflation_cells = int(self.inflation_radius / self.resolution)

        # -------------------- TF2 --------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # -------------------- ROS Setup --------------------
        self.publisher = self.create_publisher(
            OccupancyGrid, self.costmap_topic, 10
        )
        self.create_subscription(
            PointCloud2, obstacle_topic, self.cloud_callback, 10
        )

        self.get_logger().info(
            f"[CostmapBuilder] Subscribed to '{obstacle_topic}', publishing '{self.costmap_topic}' "
            f"({self.width_cells} x {self.height_cells}, res={self.resolution} m)"
        )

    # ------------------------------------------------------------------
    def cloud_callback(self, cloud: PointCloud2):
        """Main callback."""
        points = self._extract_points_tf(cloud)
        grid = self._build_grid(points)
        self.publisher.publish(grid)

    # ------------------------------------------------------------------
    def _extract_points_tf(self, cloud: PointCloud2) -> List[Tuple[float, float]]:
        """Extract obstacle points in map frame with height filtering."""
        points: List[Tuple[float, float]] = []

        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                cloud.header.frame_id,
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF transform unavailable: {ex}")
            return points

        for (x, y, z) in pc2.read_points(
            cloud, field_names=("x", "y", "z"), skip_nans=True
        ):
            if not (self.min_h <= z <= self.max_h):
                continue

            p = PointStamped()
            p.header = cloud.header
            p.point.x = x
            p.point.y = y
            p.point.z = z

            try:
                p_map = tf2_ros.do_transform_point(p, transform)
                points.append((p_map.point.x, p_map.point.y))
            except TransformException:
                continue

        return points

    # ------------------------------------------------------------------
    def _build_grid(self, points: List[Tuple[float, float]]) -> OccupancyGrid:
        """Create OccupancyGrid from obstacle points."""
        # Unknown by default
        data = np.full(
            self.width_cells * self.height_cells, -1, dtype=np.int8
        )

        obstacle_cells: Set[Tuple[int, int]] = set()

        # Mark lethal obstacles
        for x, y in points:
            gx = floor((x - self.origin_x) / self.resolution)
            gy = floor((y - self.origin_y) / self.resolution)

            if 0 <= gx < self.width_cells and 0 <= gy < self.height_cells:
                idx = gy * self.width_cells + gx
                data[idx] = 100
                obstacle_cells.add((gx, gy))

        # Inflate obstacles (single pass)
        self._inflate(data, obstacle_cells)

        # Build OccupancyGrid message
        grid = OccupancyGrid()
        grid.header.frame_id = self.frame_id
        grid.header.stamp = self.get_clock().now().to_msg()

        grid.info.resolution = self.resolution
        grid.info.width = self.width_cells
        grid.info.height = self.height_cells
        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y
        grid.info.origin.orientation.w = 1.0

        grid.data = data.tolist()
        return grid

    # ------------------------------------------------------------------
    def _inflate(self, data: np.ndarray, obstacle_cells: Set[Tuple[int, int]]):
        """Inflate lethal obstacles with lower costs."""
        r = self.inflation_cells
        r2 = r * r

        for gx, gy in obstacle_cells:
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if dx * dx + dy * dy > r2:
                        continue

                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < self.width_cells and 0 <= ny < self.height_cells:
                        idx = ny * self.width_cells + nx
                        if data[idx] != 100:
                            data[idx] = max(data[idx], 50)


# ------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = CostmapBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
