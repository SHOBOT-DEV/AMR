# shobot_robot_diagnostics

Publishes `diagnostic_msgs/DiagnosticArray` on `/diagnostics` summarizing motor temperature, IMU errors, encoder errors, camera status, and network quality.

## Inputs (topics)
- `motor_temp_topic` (`std_msgs/Float32`, default `/motor/temperature`)
- `imu_error_topic` (`std_msgs/String`, default `/imu/errors`)
- `encoder_error_topic` (`std_msgs/String`, default `/encoders/errors`)
- `camera_status_topic` (`std_msgs/String` or `std_msgs/Bool`, default `/camera/status`)
- `network_quality_topic` (`std_msgs/Float32`, default `/network/quality`, 0-100 quality)

## Parameters
- `publish_rate` (Hz, default 1.0)
- `timeout_sec` (mark WARN if no data within this time, default 2.0)
- `motor_temp_warn` / `motor_temp_error` (Celsius)
- `network_warn` / `network_error` (percent quality)

## Launch
```
ros2 launch shobot_robot_diagnostics robot_diagnostics_launch.py \
  motor_temp_warn:=70.0 motor_temp_error:=85.0 \
  network_warn:=40.0 network_error:=15.0
```

## Behavior
- Each signal is OK/WARN/ERROR depending on thresholds or error text.
- If no message arrives within `timeout_sec`, the status is WARN with "No data".
- Camera accepts either string status ("ok"/"ready" -> OK, other text -> WARN) or Bool (`true` -> OK, `false` -> fault).
