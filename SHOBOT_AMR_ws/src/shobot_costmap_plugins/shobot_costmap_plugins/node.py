#!/usr/bin/env python3
"""Simple PointCloud2-to-OccupancyGrid mapper for costmap layering."""
from math import floor
from typing import List, Tuple

import numpy as np
import rclpy
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class CostmapBuilder(Node):
    """Project incoming obstacle point clouds into a 2D occupancy grid."""

    def __init__(self):
        super().__init__("shobot_costmap_plugins")

        # Parameters
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("resolution", 0.1)
        self.declare_parameter("width", 20.0)  # meters
        self.declare_parameter("height", 20.0)  # meters
        self.declare_parameter("origin_x", -10.0)
        self.declare_parameter("origin_y", -10.0)
        self.declare_parameter("obstacle_topic", "/obstacles")
        self.declare_parameter("costmap_topic", "/shobot_costmap")
        self.declare_parameter("max_obstacle_height", 2.0)
        self.declare_parameter("min_obstacle_height", 0.05)
        self.declare_parameter("inflation_radius", 0.2)

        obstacle_topic = self.get_parameter("obstacle_topic").value
        self.costmap_topic = self.get_parameter("costmap_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.resolution = float(self.get_parameter("resolution").value)
        self.origin_x = float(self.get_parameter("origin_x").value)
        self.origin_y = float(self.get_parameter("origin_y").value)
        self.width_m = float(self.get_parameter("width").value)
        self.height_m = float(self.get_parameter("height").value)
        self.max_obstacle_height = float(self.get_parameter("max_obstacle_height").value)
        self.min_obstacle_height = float(self.get_parameter("min_obstacle_height").value)
        self.inflation_radius = float(self.get_parameter("inflation_radius").value)

        self.width_cells = int(self.width_m / self.resolution)
        self.height_cells = int(self.height_m / self.resolution)
        self.inflation_cells = max(0, int(self.inflation_radius / self.resolution))

        self.publisher = self.create_publisher(OccupancyGrid, self.costmap_topic, 10)
        self.create_subscription(PointCloud2, obstacle_topic, self.cloud_callback, 10)

        self.get_logger().info(
            f"Costmap builder listening to {obstacle_topic}, publishing {self.costmap_topic} "
            f"({self.width_cells}x{self.height_cells} @ {self.resolution} m), inflation {self.inflation_radius} m"
        )

    def cloud_callback(self, msg: PointCloud2):
        points = self._filter_points(msg)
        grid = self._build_grid(points)
        self.publisher.publish(grid)

    def _filter_points(self, cloud: PointCloud2) -> List[Tuple[float, float]]:
        points_xy = []
        for p in pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p
            if self.min_obstacle_height <= z <= self.max_obstacle_height:
                points_xy.append((x, y))
        return points_xy

    def _build_grid(self, points: List[Tuple[float, float]]) -> OccupancyGrid:
        data = np.zeros(self.width_cells * self.height_cells, dtype=np.int8)

        for x, y in points:
            gx = floor((x - self.origin_x) / self.resolution)
            gy = floor((y - self.origin_y) / self.resolution)
            if 0 <= gx < self.width_cells and 0 <= gy < self.height_cells:
                idx = int(gy * self.width_cells + gx)
                data[idx] = 100
                if self.inflation_cells > 0:
                    self._inflate(data, gx, gy)

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

    def _inflate(self, data: np.ndarray, gx: int, gy: int):
        for dx in range(-self.inflation_cells, self.inflation_cells + 1):
            for dy in range(-self.inflation_cells, self.inflation_cells + 1):
                if dx * dx + dy * dy > self.inflation_cells * self.inflation_cells:
                    continue
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < self.width_cells and 0 <= ny < self.height_cells:
                    data[int(ny * self.width_cells + nx)] = 100


def main(args=None):
    rclpy.init(args=args)
    node = CostmapBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
