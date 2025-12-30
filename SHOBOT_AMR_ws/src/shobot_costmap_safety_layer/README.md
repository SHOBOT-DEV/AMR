# shobot_costmap_safety_layer

A lightweight safety stop layer for SHOBOT AMR that issues an emergency stop
when obstacles or predefined landmarks are too close to the robot.

This node is intended as a fail-safe safety mechanism and should be used
alongside Nav2's collision monitoring and costmap layers.

## Overview

The safety layer continuously monitors:

- LaserScan data for nearby obstacles
- Optional landmark detections (via PoseArray) to enforce virtual stop zones

If any configured safety condition is violated, the node publishes a Boolean
stop signal.

## Features

- Monitors LaserScan for obstacles within a configurable distance
- Optional PoseArray landmark input for stop zones or virtual fences
- Publishes a `/safety_stop` Boolean flag
- Conservative, fail-safe behavior when sensor data is invalid
- Simple and robust design for real-time AMR safety

## Launch

```bash
ros2 launch shobot_costmap_safety_layer shobot_costmap_safety_layer_launch.py
```

## Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| `scan_topic` | string | Input LaserScan topic |
| `scan_threshold` | float | Stop if any scan range is below this value (meters) |
| `safety_stop_topic` | string | Output Boolean safety stop topic |
| `landmark_topic` | string | (Optional) PoseArray topic for landmarks |
| `landmark_stop_radius` | float | Stop if any landmark is within this radius (meters) |

## Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/safety_stop` | std_msgs/Bool | True when robot should stop immediately |

## Usage Notes

- This node is designed as a last-resort safety layer, not a planner.
- It should not replace Nav2 obstacle avoidance or collision monitoring.
- Ensure the landmark poses are expressed in the robot's local frame.
- Validate safety thresholds carefully for your robot's braking distance.
