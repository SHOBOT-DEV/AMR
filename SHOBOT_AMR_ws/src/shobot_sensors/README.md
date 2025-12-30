# shobot_sensors

Sensor processing and normalization package for **SHOBOT Autonomous Mobile Robots (AMR)**.

This package provides core sensor-side utility nodes responsible for:
- Normalizing raw sensor data
- Computing wheel-based odometry
- Monitoring battery health

The outputs of this package are designed to integrate cleanly with **localization**, **navigation (Nav2)**, **system monitoring**, and **fleet management** stacks.

---

## Overview

`shobot_sensors` contains three primary nodes:

1. **IMU Republisher**
2. **Wheel Encoder Odometry**
3. **Battery Monitor**

Each node focuses on a single responsibility and exposes clean, standardized ROS 2 interfaces.

---

## Nodes & Responsibilities

### 1. IMU Republisher (`shobot_imu_republisher`)

Normalizes incoming IMU messages to ensure compatibility with localization and state estimation pipelines.

**Responsibilities**
- Enforces a consistent `frame_id`
- Updates message timestamps
- Applies optional covariance matrices
- Optionally publishes a **static TF** (`base_link → imu_link`)

**Typical Use**
- Pre-processing IMU data before feeding into `robot_localization` (EKF)

---

### 2. Wheel Odometry (`shobot_wheel_odometry`)

Computes differential-drive wheel odometry from encoder ticks.

**Responsibilities**
- Converts left/right encoder ticks into distance
- Integrates pose `(x, y, yaw)`
- Publishes `nav_msgs/Odometry`
- Publishes dynamic TF: `odom → base_link`

**Typical Use**
- Input source for EKF (`robot_localization`)
- Dead-reckoning fallback when localization is degraded

---

### 3. Battery Monitor (`shobot_battery_monitor`)

Monitors battery health and publishes structured status and alarms.

**Responsibilities**
- Subscribes to `sensor_msgs/BatteryState`
- Publishes JSON battery status
- Publishes low / critical battery alarms
- Suppresses log spam by logging only on state change

**Battery States**
- `OK`
- `LOW`
- `CRITICAL`
- `UNKNOWN`

---

## Launch

Launch all sensor-related nodes together:

```bash
ros2 launch shobot_sensors shobot_sensors_launch.py
