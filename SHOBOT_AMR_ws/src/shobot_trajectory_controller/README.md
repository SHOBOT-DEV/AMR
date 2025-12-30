# shobot_trajectory_controller

Trajectory smoothing and acceleration limiting controller for SHOBOT Autonomous Mobile Robots (AMR).

This node filters incoming velocity commands (`cmd_vel`) and enforces **linear and angular acceleration limits** before forwarding them to the motor controller, ensuring smooth, safe, and physically realistic motion.

---

## Overview

`shobot_trajectory_controller` sits downstream of velocity arbitration (e.g., `shobot_twist_mux`) and prevents abrupt velocity changes that can cause wheel slip, mechanical stress, or unsafe robot behavior.

It is especially important for:
- Industrial AMRs
- Human-shared environments
- RL-based planners that may output aggressive commands
- Teleoperation safety

---

## Features

- Acceleration-limited velocity smoothing
- Independent linear and angular acceleration control
- Optional **instant stop** behavior for safety
- Fixed-rate deterministic update loop
- Planner-agnostic (works with nav, teleop, RL)
- Lightweight and real-time friendly

---

## Topics

### Subscribed Topics

| Topic | Type | Description |
|------|------|------------|
| `/cmd_vel_raw` | `geometry_msgs/Twist` | Incoming unfiltered velocity command |

### Published Topics

| Topic | Type | Description |
|------|------|------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Smoothed velocity command |

---

## Parameters

| Parameter | Type | Default | Description |
|---------|------|---------|------------|
| `input_topic` | string | `/cmd_vel_raw` | Input velocity topic |
| `output_topic` | string | `/cmd_vel` | Output filtered velocity topic |
| `rate_hz` | float | `30.0` | Controller update rate |
| `max_linear_accel` | float | `0.5` | Maximum linear acceleration (m/s²) |
| `max_angular_accel` | float | `1.0` | Maximum angular acceleration (rad/s²) |
| `instant_stop` | bool | `true` | Allow immediate stop on zero command |

---

## Launch & Run

### Launch using ROS 2 launch file (recommended)

```bash
ros2 launch shobot_trajectory_controller shobot_trajectory_controller_launch.py
