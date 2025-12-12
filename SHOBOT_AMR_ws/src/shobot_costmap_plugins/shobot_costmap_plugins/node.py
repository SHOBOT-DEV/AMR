#!/usr/bin/env python3
"""
CostmapBuilder
--------------
Projects PointCloud2 obstacle data into a 2D OccupancyGrid suitable for AMR local
costmap layering. Includes height filtering + inflation.
"""

from math import floor
from typing import List, Tuple

import numpy as np
import rclpy
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class CostmapBuilder(Node):
    """Convert filtered point clouds into a static-style occupancy costmap."""

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
        self.declare_parameter("max_obstacle_height", 2.0)
        self.declare_parameter("min_obstacle_height", 0.05)
        self.declare_parameter("inflation_radius", 0.2)

        # Read parameters
        obstacle_topic = self.get_parameter("obstacle_topic").value
        self.costmap_topic = self.get_parameter("costmap_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.resolution = float(self.get_parameter("resolution").value)
        self.origin_x = float(self.get_parameter("origin_x").value)
        self.origin_y = float(self.get_parameter("origin_y").value)
        self.width_m = float(self.get_parameter("width").value)
        self.height_m = float(self.get_parameter("height").value)
        self.max_h = float(self.get_parameter("max_obstacle_height").value)
        self.min_h = float(self.get_parameter("min_obstacle_height").value)
        self.inflation_radius = float(self.get_parameter("inflation_radius").value)

        # Derived parameters
        self.width_cells = int(self.width_m / self.resolution)
        self.height_cells = int(self.height_m / self.resolution)
        self.inflation_cells = int(max(0.0, self.inflation_radius / self.resolution))

        # -------------------- ROS Setup --------------------
        self.publisher = self.create_publisher(OccupancyGrid, self.costmap_topic, 10)
        self.create_subscription(PointCloud2, obstacle_topic, self.cloud_callback, 10)

        self.get_logger().info(
            f"[CostmapBuilder] Listening on '{obstacle_topic}' → publishing costmap '{self.costmap_topic}' "
            f"({self.width_cells}×{self.height_cells}, res={self.resolution} m)"
        )

    # ------------------------------------------------------------------
    def cloud_callback(self, msg: PointCloud2):
        """Main callback: filter point cloud and build occupancy grid."""
        points = self._filter_points(msg)
        grid = self._build_grid(points)
        self.publisher.publish(grid)

    # ------------------------------------------------------------------
    def _filter_points(self, cloud: PointCloud2) -> List[Tuple[float, float]]:
        """Extract valid XY points inside height bounds."""
        pts = []

        for (x, y, z) in pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
            if self.min_h <= z <= self.max_h:
                pts.append((x, y))

        return pts

    # ------------------------------------------------------------------
    def _build_grid(self, points: List[Tuple[float, float]]) -> OccupancyGrid:
        """Create occupancy grid from XY obstacle points."""
        # Start with entirely free grid
        data = np.zeros(self.width_cells * self.height_cells, dtype=np.int8)

        # Mark obstacles
        for x, y in points:
            gx = floor((x - self.origin_x) / self.resolution)
            gy = floor((y - self.origin_y) / self.resolution)

            if 0 <= gx < self.width_cells and 0 <= gy < self.height_cells:
                index = gy * self.width_cells + gx
                data[index] = 100  # Occupied

                # Inflate around obstacle
                if self.inflation_cells > 0:
                    self._inflate(data, gx, gy)

        # Build OccupancyGrid msg
        grid = OccupancyGrid()
        grid.header.frame_id = self.frame_id
        grid.header.stamp = self.get_clock().now().to_msg()

        grid.info.resolution = self.resolution
        grid.info.width = self.width_cells
        grid.info.height = self.height_cells

        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y

        grid.data = data.tolist()
        return grid

    # ------------------------------------------------------------------
    def _inflate(self, data: np.ndarray, gx: int, gy: int):
        """Inflate obstacles using circle mask around (gx, gy)."""
        r = self.inflation_cells
        r2 = r * r

        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):

                if dx*dx + dy*dy > r2:
                    continue

                nx, ny = gx + dx, gy + dy

                if 0 <= nx < self.width_cells and 0 <= ny < self.height_cells:
                    index = ny * self.width_cells + nx
                    data[index] = 100  # Mark inflated cell


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
