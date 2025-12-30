# shobot_costmap_plugins

Custom Nav2 costmap plugins for SHOBOT, including point-cloud-based obstacle
layers and occupancy grid generation for Autonomous Mobile Robots (AMRs).

## Overview

This package provides custom costmap layers that extend Nav2's costmap
framework. It is designed to integrate sensor data (e.g., LiDAR or depth
cameras) into Nav2's local and global costmaps.

Typical use cases:

- PointCloud2 to OccupancyGrid conversion
- Custom obstacle handling beyond default Nav2 layers
- Experimental cost inflation and filtering logic

## Features

- Custom Nav2 costmap layers
- PointCloud2 obstacle projection into 2D costmaps
- Height filtering and obstacle inflation
- Nav2-compatible OccupancyGrid semantics
- Easily configurable via YAML

## Launch

```bash
ros2 launch shobot_costmap_plugins shobot_costmap_plugins_launch.py
```

## Configuration

Plugin parameters are defined in the `launch/config` directory.

Common parameters include:

- Input point cloud topic
- Target frame (e.g., `map`, `odom`)
- Costmap resolution and size
- Obstacle height thresholds
- Inflation radius and cost values

Edit the YAML configuration files in `launch/config` to tune behavior.

## Requirements

- ROS 2 (Humble or newer recommended)
- Nav2 stack
- TF2 properly configured
- PointCloud2 sensor input (LiDAR or depth camera)

## Notes

- These plugins are intended for local costmap usage in Nav2.
- Ensure correct TF transforms between sensor frames and the costmap frame.
- For production systems, validate costmap behavior in simulation before
  deployment.

## Future Work

- Dynamic to static obstacle classification
- RL-based cost inflation
- Multi-sensor costmap fusion
- Fleet-level shared costmaps
