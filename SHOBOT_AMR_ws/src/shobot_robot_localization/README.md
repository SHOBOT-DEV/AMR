# shobot_robot_localization

Localization stack for **SHOBOT Autonomous Mobile Robots (AMR)** based on
**Extended Kalman Filter (EKF)** sensor fusion.

This package fuses multiple odometry and motion sources into a single,
consistent robot state suitable for navigation, mapping, and monitoring.

---

## Overview

`shobot_robot_localization` uses `robot_localization` (EKF) to fuse:

- Wheel encoder odometry
- IMU data
- Visual SLAM odometry (e.g. RTAB-Map)
- LiDAR odometry
- Camera-based visual odometry (VO)

The fused output is typically published as `/odometry/filtered` and
used by Nav2, mapping, and higher-level autonomy modules.

---

## Fused Inputs

Typical input topics include:

| Source | Topic |
|------|------|
| Wheel odometry | `/odom/wheel` |
| IMU | `/imu/data` |
| Visual SLAM | `/rtabmap/odom` |
| LiDAR odometry | `/lidar_odom` |
| Camera VO | `/vo` |
| Fallback / external odom | `/odom` |

> Not all sources must be enabled at once.  
> The EKF configuration determines which inputs are active.

---

## Frames

Standard frame configuration:

- `map` – Global reference frame
- `odom` – Local continuous odometry frame
- `base_link` – Robot base frame

Frame names are configurable via parameters.

---

## Launch

Launch the EKF localization stack:

```bash
ros2 launch shobot_robot_localization ekf_launch.py
```
## Localization Pipeline
```text
Wheel Odom   IMU   Visual SLAM   LiDAR Odom   Camera VO
     ↓        ↓        ↓             ↓            ↓
            robot_localization (EKF)
                      ↓
              /odometry/filtered
                      ↓
                    Nav2

```

