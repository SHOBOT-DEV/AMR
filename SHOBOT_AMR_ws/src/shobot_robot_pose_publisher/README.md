# shobot_robot_pose_publisher

Robot pose republisher for **SHOBOT Autonomous Mobile Robots (AMR)**.

This node converts `nav_msgs/Odometry` messages into `geometry_msgs/PoseStamped`, providing a lightweight, UI- and visualization-friendly pose stream for dashboards, monitoring tools, and RViz.

---

## Overview

`shobot_robot_pose_publisher` subscribes to an odometry topic and republishes only the **pose** component as a `PoseStamped` message.

It ensures:
- Safe timestamp handling
- Optional frame override
- Sensor-friendly QoS for real-time visualization

---

## Topics

### Subscribed Topics

| Topic | Type | Description |
|------|------|------------|
| `/odom` (configurable) | `nav_msgs/Odometry` | Source odometry |

### Published Topics

| Topic | Type | Description |
|------|------|------------|
| `/robot_pose` (configurable) | `geometry_msgs/PoseStamped` | Republished robot pose |

---

## Parameters

| Parameter | Type | Default | Description |
|---------|------|---------|------------|
| `odom_topic` | string | `/odom` | Input odometry topic |
| `pose_topic` | string | `/robot_pose` | Output pose topic |
| `pose_frame` | string | `""` | Override frame ID (empty = inherit from odometry) |

---

## Timestamp & Frame Handling

- If the incoming odometry message has a valid timestamp, it is preserved.
- If the timestamp is missing (`0`), the current ROS time is used.
- The output `frame_id`:
  - Inherits from odometry by default
  - Can be overridden via `pose_frame`

---

## Usage

### Run standalone

```bash
   ros2 run shobot_robot_pose_publisher shobot_robot_pose_publisher_node
```

```bash
  ros2 run shobot_robot_pose_publisher shobot_robot_pose_publisher_node \
  --ros-args -p odom_topic:=/odom -p pose_topic:=/robot_pose -p pose_frame:=map

```

```text
 Wheel / EKF Odometry
        ↓
   nav_msgs/Odometry
        ↓
shobot_robot_pose_publisher
        ↓
geometry_msgs/PoseStamped
        ↓
     UI / RViz

```