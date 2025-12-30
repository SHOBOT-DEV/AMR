# shobot_system_monitor

System heartbeat and liveness monitoring package for SHOBOT Autonomous Mobile Robots (AMR).

This package publishes a **JSON-encoded heartbeat message** on `/system/heartbeat`, containing:
- System health metrics (CPU, memory, load averages)
- Liveness status of critical **sensor topics**
- Liveness status of critical **node / pipeline topics**

It is designed for **robot safety**, **fleet monitoring**, and **remote diagnostics**.

---

## Overview

`shobot_system_monitor` continuously monitors the robot’s internal health and ROS topic activity.  
It detects missing or stalled components and reports overall system status in a single, structured message.

Typical use cases include:
- Safety supervisors
- Fleet managers
- Remote dashboards
- Watchdog / recovery systems

---

## Features

- Periodic system heartbeat publication
- CPU, memory, and load average monitoring
- Topic-based liveness checks with timeouts
- Sensor and node monitoring separation
- JSON output (easy integration with web / cloud systems)
- Optional `psutil` support for accurate system metrics
- ROS 2-native and launch-file configurable

---

## Published Topics

| Topic | Type | Description |
|------|------|------------|
| `/system/heartbeat` | `std_msgs/String` | JSON-encoded system heartbeat |

---

## Parameters

| Parameter | Type | Default | Description |
|---------|------|---------|------------|
| `heartbeat_topic` | string | `/system/heartbeat` | Output heartbeat topic |
| `rate_hz` | float | `1.0` | Heartbeat publish rate |
| `sensor_topics` | string[] | `[]` | Sensor topics to monitor |
| `node_topics` | string[] | `[]` | Node / pipeline topics to monitor |

### Topic Watch Format

Each entry in `sensor_topics` or `node_topics` must follow:


- `name` – logical identifier
- `topic` – ROS topic name
- `timeout_sec` – max allowed message age
- `type` *(optional)* – ROS message type (default: `std_msgs/msg/String`)

---

## Launch Example

```bash
ros2 launch shobot_system_monitor system_heartbeat_launch.py \
  sensor_topics:="[
    lidar:/scan:1.0:sensor_msgs/msg/LaserScan,
    imu:/imu/data:1.0:sensor_msgs/msg/Imu
  ]" \
  node_topics:="[
    nav:/navigation_status:2.0
  ]"
```
# Heartbeat Payload Format
````
{
  "timestamp_ns": 1700000000000000000,
  "cpu_percent": 12.3,
  "mem_percent": 45.6,
  "load_avg": [0.45, 0.50, 0.60],
  "sensors": [
    {
      "name": "lidar",
      "topic": "/scan",
      "alive": true,
      "age_sec": 0.2,
      "timeout_sec": 1.0
    }
  ],
  "nodes": [
    {
      "name": "nav",
      "topic": "/navigation_status",
      "alive": true,
      "age_sec": 0.5,
      "timeout_sec": 2.0
    }
  ],
  "ok": true
}
