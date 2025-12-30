# shobot_tf_static

Static TF2 broadcaster for SHOBOT Autonomous Mobile Robots (AMR).

This package publishes all **required static transforms** between the robot base and onboard sensors, using a **parameter-driven** and **Nav2-safe** TF tree configuration.

---

## Overview

`shobot_tf_static` provides a centralized, configurable static transform publisher for SHOBOT.  
It replaces multiple ad-hoc `static_transform_publisher` invocations with a **single, robust ROS 2 node**.

By default, it publishes:
- `base_link → laser`
- `base_link → imu`
- `base_link → camera_frame`

Optionally:
- `map → odom` (disabled by default to avoid TF conflicts with Nav2)

---

## Published Transforms

| Parent Frame | Child Frame | Description |
|--------------|------------|-------------|
| `base_link` | `laser` | LiDAR mounting transform |
| `base_link` | `imu` | IMU mounting transform |
| `base_link` | `camera_frame` | Camera mounting transform |
| `map` | `odom` | Global localization transform (optional) |

---

## Parameters

All transforms are configured via ROS 2 parameters (typically through YAML).

### Frame Parameters
| Parameter | Default | Description |
|---------|---------|-------------|
| `base_frame` | `base_link` | Robot base frame |
| `laser_frame` | `laser` | LiDAR frame |
| `imu_frame` | `imu` | IMU frame |
| `camera_frame` | `camera_frame` | Camera frame |
| `map_frame` | `map` | Map frame |
| `odom_frame` | `odom` | Odometry frame |

### Transform Parameters

Each transform uses:
- Translation `[x, y, z]` in **meters**
- Rotation `[roll, pitch, yaw]` in **radians**

| Parameter | Description |
|---------|-------------|
| `base_to_laser_translation` | Base → LiDAR translation |
| `base_to_laser_rpy` | Base → LiDAR rotation |
| `base_to_imu_translation` | Base → IMU translation |
| `base_to_imu_rpy` | Base → IMU rotation |
| `base_to_camera_translation` | Base → Camera translation |
| `base_to_camera_rpy` | Base → Camera rotation |
| `map_to_odom_translation` | Map → Odom translation |
| `map_to_odom_rpy` | Map → Odom rotation |
| `publish_map_to_odom` | Enable map → odom transform |

---

## Example Configuration

```yaml
base_frame: base_link

laser_frame: laser
base_to_laser_translation: [0.25, 0.0, 0.20]
base_to_laser_rpy: [0.0, 0.0, 0.0]

imu_frame: imu
base_to_imu_translation: [0.0, 0.0, 0.10]
base_to_imu_rpy: [0.0, 0.0, 0.0]

camera_frame: camera_frame
base_to_camera_translation: [0.15, 0.0, 0.30]
base_to_camera_rpy: [0.0, 0.0, 0.0]

publish_map_to_odom: false
