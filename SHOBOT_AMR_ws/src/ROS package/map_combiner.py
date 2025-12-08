#!/usr/bin/env python3
"""Combine 2D lidar maps with 3D depth obstacles and expose ROS2 services."""
import numpy as np
import sensor_msgs.point_cloud2 as pc2lib
import tf2_geometry_msgs
import tf2_ros
from copy import deepcopy
from numpy.linalg import norm
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.time import Time
from rs_lidar_fusion.srv import map_combiner as MapCombinerSrv
from rs_lidar_fusion.srv import obstacle as ObstacleSrv
from sensor_msgs.msg import PointCloud2, PointCloud
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64
from sklearn.cluster import DBSCAN


class MapCombiner(Node):
    """Handles map fusion, height filtering, and obstacle registration."""

    def __init__(self):
        super().__init__('map_combiner')

        latched_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.lidar_map_client = self.create_client(GetMap, '/dynamic_map')
        while not self.lidar_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /dynamic_map service...')

        self.map_2d = self._fetch_initial_map()
        self.combined_map_data = list(self.map_2d.data) if self.map_2d else []

        self.height = 0.2
        self.height_pub = self.create_publisher(Float64, '/height', qos_profile=latched_qos)
        self.height_pub.publish(Float64(data=self.height))

        self.map_publisher = self.create_publisher(OccupancyGrid, 'combined_map', qos_profile=latched_qos)
        self.lidar_map_sub = self.create_subscription(OccupancyGrid, 'map', self.lidar_map_callback, 10)
        self.create_subscription(PointCloud2, '/rtabmap/cloud_obstacles', self.cloud_callback, 10)
        self.create_subscription(PointCloud2, '/camera/depth/color/points', self.camera_cloud_callback, 10)

        self.map_3d = None
        self.camera_cloud = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_service(MapCombinerSrv, 'map_combiner/set_height', self.process_depth_pointcloud)
        self.create_service(ObstacleSrv, 'map_combiner/register_obstacle', self.register_obstacle)

        self.get_logger().info('MapCombiner node ready (ROS2).')

    def _fetch_initial_map(self):
        request = GetMap.Request()
        future = self.lidar_map_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            return future.result().map
        self.get_logger().warn('Failed to fetch initial map; waiting for first topic update.')
        return None

    def getCoord(self, x, y):
        """Convert metric coordinates to map grid indices."""
        if not self.map_2d:
            return (0, 0)
        map_resolution = self.map_2d.info.resolution
        map_origin = self.map_2d.info.origin.position
        map_x = int((x - map_origin.x) / map_resolution)
        map_y = int((y - map_origin.y) / map_resolution)
        return (map_x, map_y)

    def lidar_map_callback(self, data):
        """Cache new 2D lidar map and trigger a fusion pass."""
        self.map_2d = data
        self.combined_map_data = list(self.map_2d.data)
        self.process_depth_pointcloud(None, MapCombinerSrv.Response())

    def cloud_callback(self, msg):
        self.map_3d = msg

    def camera_cloud_callback(self, msg):
        self.camera_cloud = msg

    def process_depth_pointcloud(self, req, resp):
        """Fuse height-filtered depth cloud into the occupancy grid."""
        if req is not None:
            self.height = req.scan_height
            self.height_pub.publish(Float64(data=self.height))

        if not self.map_2d:
            self.get_logger().warn('No 2D map available yet; skipping fusion.')
            return MapCombinerSrv.Response()

        if not self.map_3d:
            self.get_logger().warn('No 3D cloud yet; skipping fusion.')
            return MapCombinerSrv.Response()

        combined = list(self.map_2d.data)

        # Ensure the rgb field reads as float32
        self.map_3d.fields[-1].datatype = 6
        pointcloud_data = pc2lib.read_points(self.map_3d, skip_nans=True, field_names=('x', 'y', 'z', 'rgb'))

        for point in pointcloud_data:
            if self.height > point[2] > 0.05:
                x, y = self.getCoord(point[0], point[1])
                idx = y * self.map_2d.info.width + x
                if 0 <= idx < len(combined):
                    combined[idx] = 100

        self.publish_map(combined)
        return MapCombinerSrv.Response()

    def register_obstacle(self, req, resp):
        """Register a camera-detected obstacle into the map at a transformed pose."""
        self.get_logger().info('register_obstacle service called')

        if not self.map_2d:
            self.get_logger().warn('No map available to register obstacle.')
            return ObstacleSrv.Response()

        try:
            trans = self.tf_buffer.lookup_transform(
                'map',
                'camera_depth_optical_frame',
                Time())
        except tf2_ros.LookupException:
            self.get_logger().warn('Transform map <- camera_depth_optical_frame unavailable.')
            return ObstacleSrv.Response()

        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = 'camera_depth_optical_frame'
        pt.point.x, pt.point.y, pt.point.z = req.x, req.y, req.z
        pt = tf2_geometry_msgs.do_transform_point(pt, trans)
        x, y, z = pt.point.x, pt.point.y, pt.point.z

        if not self.camera_cloud:
            self.get_logger().warn('No camera depth cloud available to cluster obstacle.')
            return ObstacleSrv.Response()

        depth_cam_data = do_transform_cloud(self.camera_cloud, trans)
        pointcloud_data = [p for p in pc2lib.read_points(depth_cam_data, skip_nans=True, field_names=('x', 'y', 'z'))
                           if 0.05 < p[2] < self.height]

        if not pointcloud_data:
            self.get_logger().warn('No depth points within height bounds for obstacle.')
            return ObstacleSrv.Response()

        pointcloud_data = np.array(pointcloud_data)
        labels = DBSCAN(eps=0.2, min_samples=10).fit(pointcloud_data).labels_

        obstacle_label = None
        for i in range(len(labels)):
            if norm(pointcloud_data[i] - [x, y, z]) < 0.1:
                obstacle_label = labels[i]
                break

        if obstacle_label is None:
            self.get_logger().warn('Obstacle cluster not found near requested point.')
            return ObstacleSrv.Response()

        obstacle_points = pointcloud_data[np.where(labels == obstacle_label)]

        combined = list(self.map_2d.data) if self.map_2d else list(self.combined_map_data)
        for point in obstacle_points:
            gx, gy = self.getCoord(point[0], point[1])
            idx = gy * self.map_2d.info.width + gx
            if 0 <= idx < len(combined):
                combined[idx] = 100

        self.publish_map(combined)
        return ObstacleSrv.Response()

    def publish_map(self, map_data):
        """Publish a fresh OccupancyGrid with updated obstacle markings."""
        new_map = OccupancyGrid()
        new_map.header = deepcopy(self.map_2d.header)
        new_map.header.stamp = self.get_clock().now().to_msg()
        new_map.info = deepcopy(self.map_2d.info)
        new_map.info.map_load_time = self.get_clock().now().to_msg()
        new_map.data = tuple(map_data)

        self.map_publisher.publish(new_map)


def main():
    rclpy.init()
    node = MapCombiner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
