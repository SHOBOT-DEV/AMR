# shobot_pointcloud_filter

Lightweight PointCloud2 filtering and forwarding node for SHOBOT AMR.

This node performs **optional height (Z-axis) filtering**, **frame ID override**, and **safe pass-through** of high-rate 3D point clouds. It is designed for LiDAR, depth cameras, and other PointCloud2 sources used by Nav2, costmaps, and perception pipelines.

---

## Features

- Pass-through forwarding of `sensor_msgs/PointCloud2`
- Optional **height (Z) filtering**
- Optional **frame_id override**
- Sensor-friendly QoS (BEST_EFFORT, depth=1)
- Preserves original PointCloud2 metadata
- Safe timestamp correction if missing

---

## Topics

### Subscribed
- **`input_topic`** (`sensor_msgs/PointCloud2`)  
  Default: `/points_raw`

### Published
- **`output_topic`** (`sensor_msgs/PointCloud2`)  
  Default: `/points_filtered`

---

## Parameters

| Parameter | Type | Default | Description |
|--------|------|---------|-------------|
| `input_topic` | string | `/points_raw` | Input point cloud topic |
| `output_topic` | string | `/points_filtered` | Output filtered cloud |
| `enable_height_filter` | bool | `false` | Enable Z-axis filtering |
| `min_z` | float | `-999.0` | Minimum Z value (meters) |
| `max_z` | float | `999.0` | Maximum Z value (meters) |
| `frame_id` | string | `""` | Override frame_id (empty = inherit) |

---

## Usage

### Basic pass-through
```bash
    ros2 run shobot_pointcloud_filter shobot_pointcloud_filter
```

```bash
    ros2 run shobot_pointcloud_filter shobot_pointcloud_filter \
      --ros-args \
      -p enable_height_filter:=true \
      -p min_z:=0.05 \
      -p max_z:=1.5

```

```bash
    ros2 run shobot_pointcloud_filter shobot_pointcloud_filter \
      --ros-args \
      -p frame_id:=base_link

```
## Pipeline
```text
LiDAR / Depth Camera
        ↓
/points_raw
        ↓
shobot_pointcloud_filter
        ↓
/points_filtered
        ↓
Nav2 / Costmap / Perception

```