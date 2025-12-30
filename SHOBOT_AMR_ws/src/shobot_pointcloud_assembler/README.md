# shobot_pointcloud_assembler

Assembles multiple incoming `sensor_msgs/PointCloud2` streams into a single cloud for downstream perception, mapping, and navigation.

Designed for SHOBOT AMR systems where data from multiple 3D sensors (front/rear LiDARs, depth cameras) must be fused before being consumed by Nav2, SLAM, or obstacle detection pipelines.

## What this node does
- Subscribes to multiple `PointCloud2` topics.
- Stores the latest cloud from each input topic.
- Verifies field consistency (x, y, z, intensity, datatype, offsets).
- Concatenates all valid points into one output `PointCloud2`.
- Publishes the combined cloud when all inputs are available.
- Pure spatial merge only (no filtering, downsampling, or time sync).

## Typical use cases
```
Front LiDAR ─┐
             ├──▶ shobot_pointcloud_assembler ───▶ /points_assembled ─▶ Nav2 / SLAM
Rear LiDAR  ─┘
```

With preprocessing:
```
Front Camera ─▶ pointcloud_filter ─┐
                                   ├──▶ shobot_pointcloud_assembler ─▶ costmap
Rear Camera  ─▶ pointcloud_filter ─┘
```

## Parameters
- `input_topics` (string[], default `["/points_filtered"]`): PointCloud2 topics to merge.
- `output_topic` (string, default `/points_assembled`): Output combined point cloud topic.
- `frame_id` (string, default `""`): Override frame ID (empty = inherit input frame).
- `keep_source_timestamps` (bool, default `false`): Use timestamp from first input cloud instead of current time.

## Usage
**Launch example**
```bash
    ros2 launch shobot_pointcloud_assembler pointcloud_assembler_launch.py \
      input_topics:="[/points_front, /points_rear]" \
      output_topic:=/points_assembled \
      frame_id:=base_link
```

**Minimal YAML example**
```yaml
shobot_pointcloud_assembler:
  ros__parameters:
    input_topics:
      - /points_front
      - /points_rear
    output_topic: /points_assembled
    frame_id: base_link
    keep_source_timestamps: false
```

## Important constraints
- All input point clouds must have identical fields:
  - Same field names (x, y, z, intensity, etc.).
  - Same datatype.
  - Same offset and count.
- If fields do not match, the node will not publish and will log an error.
