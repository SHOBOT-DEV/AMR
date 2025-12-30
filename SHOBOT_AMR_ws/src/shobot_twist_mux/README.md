# shobot_twist_mux

Priority-based velocity command multiplexer for SHOBOT Autonomous Mobile Robots (AMR).

This node arbitrates multiple `/cmd_vel` sources (e.g., safety, teleoperation, navigation, RL planners) and publishes a single safe, deterministic velocity command based on **priority**, **timeout**, and **safety override** rules.

---

## Overview

`shobot_twist_mux` continuously selects the **highest-priority valid velocity command** from multiple sources and forwards it to the robot controller.

It ensures:
- Safe arbitration between autonomy and human control
- Immediate reaction to safety events
- Automatic stopping on stale commands
- Deterministic and fail-safe behavior

This node replaces or extends the standard ROS `twist_mux` with **timeout handling** and **hard safety stop support**.

---

## Features

- Priority-based arbitration of multiple velocity sources
- Per-source timeout enforcement
- Hard safety stop override (zero velocity)
- Deterministic and fail-safe output behavior
- Fully configurable via ROS 2 parameters
- Lightweight and planner-agnostic
- Designed for industrial AMR systems

---

## Topics

### Subscribed Topics

| Topic | Type | Description |
|------|------|------------|
| `/cmd_vel/<source>` | `geometry_msgs/Twist` | Velocity commands from each source |
| `/safety_stop` | `std_msgs/Bool` | Hard safety stop signal |

### Published Topics

| Topic | Type | Description |
|------|------|------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Selected velocity command |

---

## Parameters

| Parameter | Type | Default | Description |
|---------|------|---------|------------|
| `sources` | string[] | `["safety","teleop","nav"]` | List of velocity sources |
| `priorities_json` | string (JSON) | see below | Source priority mapping |
| `timeouts_json` | string (JSON) | see below | Per-source timeout (seconds) |
| `topics_json` | string (JSON) | see below | Input topic per source |
| `output_topic` | string | `/cmd_vel` | Output velocity topic |
| `rate_hz` | float | `20.0` | Arbitration loop rate |
| `safety_stop_topic` | string | `/safety_stop` | Safety stop input topic |

### Default JSON Configuration

```json
{
  "priorities_json": {
    "safety": 100,
    "teleop": 50,
    "nav": 10
  },
  "timeouts_json": {
    "safety": 0.5,
    "teleop": 0.5,
    "nav": 1.0
  },
  "topics_json": {
    "safety": "/cmd_vel/safety",
    "teleop": "/cmd_vel/teleop",
    "nav": "/cmd_vel/nav"
  }
}
