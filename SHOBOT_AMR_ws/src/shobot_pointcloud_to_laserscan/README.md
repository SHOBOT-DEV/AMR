# shobot_pointcloud_to_laserscan

Converts **3D `sensor_msgs/PointCloud2`** data into **2D `sensor_msgs/LaserScan`** messages by projecting points onto a planar slice and selecting the **minimum range per angular bin**.

This node is designed for **AMR navigation and safety**, and is fully compatible with **Nav2 costmaps**, obstacle layers, and safety controllers.

---

## Features

- 3D → 2D projection using configurable height slicing
- Per-angle minimum range selection (closest obstacle)
- Configurable angular limits and resolution
- Range filtering (`range_min`, `range_max`)
- Sensor-friendly QoS (BEST_EFFORT)
- Lightweight and deterministic
- Suitable for 3D LiDARs and depth cameras

---

## Typical Pipeline

```text
3D LiDAR / Depth Camera
        ↓
   PointCloud2
        ↓
shobot_pointcloud_to_laserscan
        ↓
     LaserScan
        ↓
   Nav2 / Costmap / Safety
```
## Launch and Run
```bash
    ros2 launch shobot_pointcloud_to_laserscan pointcloud_to_laserscan_launch.py \
      input_topic:=/points \
      output_topic:=/scan_from_points \
      scan_frame:=base_scan

```