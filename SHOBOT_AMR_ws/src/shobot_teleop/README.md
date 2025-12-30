# shobot_teleop

Teleoperation input package for SHOBOT Autonomous Mobile Robots (AMR).

This package provides a **keyboard-based teleoperation node** that publishes velocity commands to a dedicated teleop channel (`cmd_vel/teleop`), typically consumed by a velocity multiplexer (`shobot_twist_mux`).

---

## Overview

`shobot_teleop` allows a human operator to manually control the robot using a keyboard.  
It supports **model-based velocity limits**, **smooth velocity profiling**, and **safe shutdown behavior**.

This node is intended for:
- Manual override
- Testing and debugging
- Recovery and maintenance operations
- Human-in-the-loop demonstrations

---

## Features

- Keyboard-based velocity control
- Model-specific velocity limits (`burger`, `waffle`, `waffle_pi`)
- Smooth acceleration profiling
- Non-blocking terminal input
- Safe stop on exit (zero velocity)
- Parameterized output topic and publish rate

---

## Topics

### Published Topics

| Topic | Type | Description |
|------|------|------------|
| `/cmd_vel/teleop` | `geometry_msgs/Twist` | Teleoperation velocity commands |

*(Topic can be remapped via parameters or launch files.)*

---

## Controls

| Key | Action |
|----|-------|
| `w` | Increase forward linear velocity |
| `x` | Decrease forward linear velocity |
| `a` | Increase angular velocity (left) |
| `d` | Decrease angular velocity (right) |
| `space` or `s` | Immediate stop |
| `CTRL + C` | Exit teleop |

---

## Parameters

| Parameter | Type | Default | Description |
|---------|------|---------|------------|
| `teleop_topic` | string | `/cmd_vel/teleop` | Output Twist topic |
| `model` | string | `burger` | Velocity limit preset |
| `rate_hz` | float | `20.0` | Publish rate |

### Supported Models

| Model | Max Linear (m/s) | Max Angular (rad/s) |
|------|-----------------|---------------------|
| `burger` | 0.8 | 0.6 |
| `waffle` / `waffle_pi` | 0.26 | 1.82 |

---

## Launch & Run

### Run standalone

```bash
ros2 run shobot_teleop shobot_teleop_keyboard
