# shobot_status_aggregator

Status aggregation package for SHOBOT Autonomous Mobile Robots (AMR).

This node collects status messages from multiple subsystems (navigation, mission control, servers, etc.) and publishes a **single JSON-encoded status summary** suitable for dashboards, fleet managers, and external monitoring systems.

---

## Overview

`shobot_status_aggregator` subscribes to multiple ROS 2 status topics and continuously aggregates their latest values into one consolidated status message.

It provides a **single source of truth** for overall robot state, simplifying:
- UI dashboards
- Fleet monitoring
- Logging and diagnostics
- External system integration

---

## Features

- Aggregates multiple subsystem status topics
- Publishes a single JSON summary
- Configurable input topics via parameters
- Fixed-rate publishing
- Robust startup behavior (`unknown` until first update)
- Lightweight and low overhead

---

## Topics

### Subscribed Topics

Configured via parameters.

| Topic | Type | Description |
|------|------|------------|
| *(configurable)* | `std_msgs/String` | Subsystem status topics |

### Published Topics

| Topic | Type | Description |
|------|------|------------|
| `/system_status` | `std_msgs/String` | JSON-encoded aggregated status |

---

## Parameters

| Parameter | Type | Default | Description |
|---------|------|---------|------------|
| `status_topics` | string[] | see below | List of status topics to aggregate |
| `output_topic` | string | `/system_status` | Output aggregated status topic |
| `rate_hz` | float | `2.0` | Publish rate |

### Default `status_topics`

```yaml
status_topics:
  - /mission_status
  - /navigation_status
  - /navigation_server/status
  - /mission_handler_status

```

```bash
   ros2 run shobot_status_aggregator shobot_status_aggregator_node
```

```bash
   ros2 launch shobot_status_aggregator status_aggregator_launch.py
```

```aiignore
{
  "timestamp_ns": 1700000000000000000,
  "count": 4,
  "statuses": {
    "/mission_status": "IDLE",
    "/navigation_status": "ACTIVE",
    "/navigation_server/status": "OK",
    "/mission_handler_status": "READY"
  }
}

```