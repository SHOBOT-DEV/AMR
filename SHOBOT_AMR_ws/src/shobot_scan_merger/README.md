# shobot_scan_merger

LaserScan aggregation node for SHOBOT Autonomous Mobile Robots (AMR).

This package merges multiple `sensor_msgs/LaserScan` topics into a **single combined scan** by taking the **minimum valid range per angle index**.  
It is intended for robots with multiple lidars (e.g., front + rear) that already share a common scan geometry.

---

## Overview

`shobot_scan_merger` subscribes to multiple LaserScan topics and produces one merged scan that represents the closest obstacle at each angular index.

This node is typically used to:
- Provide a single scan input to Nav2
- Extend obstacle coverage (front + rear lidars)
- Simplify costmap configuration

---

## Features

- Merges multiple LaserScan topics into one
- Takes the **minimum valid range** per index
- Handles `inf` and `NaN` values correctly
- Preserves scan geometry (angles, increments, timing)
- Fixed-rate publishing for stable downstream consumption
- Sensor-QoS compatible

---

## Assumptions & Limitations ⚠️

This node assumes that **all input scans**:

- Have the **same `angle_min`**
- Have the **same `angle_max`**
- Have the **same `angle_increment`**
- Are already expressed in the **same frame**

If these conditions are **not met**, scans will be ignored and a warning will be logged.

> ❗ If your lidars have different fields of view, resolutions, or frames, use a **PointCloud-based merger** instead.

---

## Topics

### Subscribed Topics
Configured via parameters.

| Topic | Type | Description |
|------|------|------------|
| *(configurable)* | `sensor_msgs/LaserScan` | Input laser scans |

### Published Topics

| Topic | Type | Description |
|------|------|------------|
| `/scan_merged` | `sensor_msgs/LaserScan` | Merged laser scan |

---

## Parameters

| Parameter | Type | Default | Description |
|---------|------|---------|------------|
| `input_topics` | string[] | `[/scan_front, /scan_rear]` | List of input LaserScan topics |
| `output_topic` | string | `/scan_merged` | Output merged scan topic |
| `frame_id` | string | `base_laser` | Frame ID for merged scan |
| `publish_rate_hz` | float | `10.0` | Publish rate |

---

## Launch & Run

### Run directly

```bash
   ros2 run shobot_scan_merger shobot_scan_merger_node


```

``` bash
  ros2 launch shobot_scan_merger shobot_scan_merger_launch.py

```

``` bash
   ros2 launch shobot_scan_merger shobot_scan_merger_launch.py \
  input_topics:="[/scan_front, /scan_rear, /scan_side]" \
  output_topic:=/scan

```

## Typical Navigation Pipeline

```text
Front Lidar      Rear Lidar
     ↓               ↓
   LaserScan      LaserScan
        \         /
         shobot_scan_merger
                 ↓
            /scan_merged
                 ↓
               Nav2
```

