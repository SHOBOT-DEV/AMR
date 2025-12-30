# shobot_zone_management

Manages operational, restricted, and special-purpose zones for SHOBOT navigation by monitoring the robot’s pose and identifying which predefined zones the robot is currently inside.

This node is commonly used for **safety enforcement**, **speed regulation**, **mission control**, and **human-aware navigation** in AMR systems.

---

## Overview

`shobot_zone_management` subscribes to the robot’s pose and evaluates it against a set of configured **circular zones**.  
It publishes the **active zone(s)** whenever the robot enters or exits a zone.

Typical use cases include:
- Speed limiting near humans or workstations
- Restricted-area avoidance
- Docking / charging area detection
- Safety stop triggering
- Reward shaping for RL-based navigation

---

## Features

- Define **allowed, restricted, or special-purpose zones**
- Supports **multiple overlapping zones**
- Publishes zone changes only (no message spam)
- Zones configurable via **ROS 2 parameters**
- Lightweight and planner-agnostic
- Safe defaults if no configuration is provided

---

## Topics

### Subscribed Topics
| Topic | Type | Description |
|-----|------|------------|
| `/robot_pose` | `geometry_msgs/PoseStamped` | Robot’s current pose (from localization or odometry) |

### Published Topics
| Topic | Type | Description |
|------|------|------------|
| `/zone_status` | `std_msgs/String` | Comma-separated list of active zones or `none` |

---

## Parameters

| Parameter | Type | Default | Description |
|---------|------|---------|------------|
| `pose_topic` | string | `/robot_pose` | Pose topic to subscribe to |
| `zone_topic` | string | `/zone_status` | Output topic for active zones |
| `zones_yaml` | string | `""` | YAML string defining circular zones |

---

## Zone Configuration Format

Zones are defined using a YAML string passed via the `zones_yaml` parameter.

### Example
```yaml
zones_yaml: |
  - name: restricted_area
    x: 2.0
    y: 1.5
    r: 1.0
  - name: charging_zone
    x: -1.0
    y: 0.5
    r: 0.8
