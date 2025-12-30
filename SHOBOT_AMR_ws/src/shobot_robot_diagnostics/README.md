# shobot_robot_diagnostics

Robot health diagnostics aggregator for **SHOBOT Autonomous Mobile Robots (AMR)**.

This node publishes a consolidated
`diagnostic_msgs/DiagnosticArray` on `/diagnostics`, summarizing the
health of key robot subsystems such as motors, IMU, encoders, camera, and network.

The output is compatible with standard ROS diagnostic tools
(e.g. `rqt_robot_monitor`) and higher-level system monitors or FSMs.

---

## Overview

`shobot_robot_diagnostics` subscribes to multiple telemetry and status topics,
evaluates them against configurable thresholds and timeouts, and reports:

- **OK** – normal operation
- **WARN** – degraded or missing data
- **ERROR** – fault or critical condition

Each signal is monitored independently and combined into a single diagnostics stream.

---

## Published Topics

| Topic | Type | Description |
|------|------|-------------|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Aggregated robot diagnostics |

---

## Input Topics

| Parameter | Message Type | Default Topic | Description |
|---------|--------------|---------------|-------------|
| `motor_temp_topic` | `std_msgs/Float32` | `/motor/temperature` | Motor temperature (°C) |
| `imu_error_topic` | `std_msgs/String` | `/imu/errors` | IMU error text (empty = OK) |
| `encoder_error_topic` | `std_msgs/String` | `/encoders/errors` | Encoder error text (empty = OK) |
| `camera_status_topic` | `std_msgs/String` or `std_msgs/Bool` | `/camera/status` | Camera health/status |
| `network_quality_topic` | `std_msgs/Float32` | `/network/quality` | Network quality (0–100 %) |

> ⚠️ A ROS topic may have **only one message type**.  
> Select the camera topic type using the `camera_status_type` parameter.

---

## Parameters

### General

| Parameter | Type | Default | Description |
|---------|------|---------|-------------|
| `publish_rate` | float | `1.0` | Diagnostics publish rate (Hz) |
| `timeout_sec` | float | `2.0` | Data staleness timeout (seconds) |

---

### Motor Temperature

| Parameter | Default | Description |
|---------|---------|-------------|
| `motor_temp_warn` | `70.0` | WARN threshold (°C) |
| `motor_temp_error` | `85.0` | ERROR threshold (°C) |

---

### Camera Status

| Parameter | Default | Description |
|---------|---------|-------------|
| `camera_status_type` | `"string"` | `"string"` or `"bool"` |

**Interpretation**
- String: `"ok"`, `"ready"` → OK; other non-empty text → WARN
- Bool: `true` → OK; `false` → WARN

---

### Network Quality

| Parameter | Default | Description |
|---------|---------|-------------|
| `network_warn` | `40.0` | WARN threshold (%) |
| `network_error` | `15.0` | ERROR threshold (%) |

---

## Diagnostic Behavior

- Each subsystem produces one `DiagnosticStatus`
- Levels are determined as follows:
  - **ERROR** – fault condition or critical threshold exceeded
  - **WARN** – degraded quality or stale/missing data
  - **OK** – normal operation
- If no message is received within `timeout_sec`, the status becomes **WARN**
  with message `"No data"`
- Diagnostic key–value pairs include:
  - Current value (e.g. temperature, network quality)
  - Data age in seconds

---

## Launch

Launch using the provided launch file:

```bash
  ros2 launch shobot_robot_diagnostics robot_diagnostics_launch.py \
      motor_temp_warn:=70.0 \
      motor_temp_error:=85.0 \
      network_warn:=40.0 \
      network_error:=15.0
```
## Pipeline
```text
Sensors / Drivers / Telemetry
        ↓
shobot_robot_diagnostics
        ↓
/diagnostics
        ↓
rqt_robot_monitor / System FSM / Fleet UI

```