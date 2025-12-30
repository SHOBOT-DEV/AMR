# shobot_docking

An action-based docking controller for SHOBOT AMR that performs the final
approach and alignment with a charging station.

This package is typically invoked after navigation has brought the robot close
to the dock.

## Overview

`shobot_docking` provides a ROS 2 action server that executes a docking
maneuver when requested. It is designed to integrate with mission control,
Nav2, and perception-based dock detection.

## Features

- Provides a dock action server
- Executes a controlled docking maneuver
- Supports timeout-based failure handling
- Can be triggered automatically by mission control or manually via action
  client
- Designed for modular integration with dock perception nodes

## Launch

```bash
ros2 launch shobot_docking shobot_docking_launch.py
```

## Dock Action

Action definition:

`shobot_docking/action/Dock.action`

### Action Goal

| Field | Type | Description |
| --- | --- | --- |
| `start` | bool | Start docking when true |

### Action Feedback

| Field | Type | Description |
| --- | --- | --- |
| `distance` | float | Distance traveled during docking (meters) |

### Action Result

| Field | Type | Description |
| --- | --- | --- |
| `success` | bool | True if docking completed successfully |

## Parameters

Key parameters are configured via launch files and YAML:

- `cmd_vel_topic` - Velocity command topic
- `odom_topic` - Odometry input topic
- `dock_speed` - Linear docking speed (m/s)
- `dock_distance` - Target distance to travel during docking (m)
- `timeout_sec` - Maximum docking duration (seconds)

Refer to the launch files for full parameter definitions.

## Typical Workflow

- Nav2 navigates the robot near the charging dock
- Dock detection nodes confirm dock presence
- Mission control sends a dock action goal
- Docking controller performs final approach
- Result is returned to mission control

## Usage Notes

- This node performs a straight-line final approach by design
- Dock alignment and perception should be handled upstream
- Always validate docking behavior in simulation before deployment
- Use conservative speed and timeout values for safety
