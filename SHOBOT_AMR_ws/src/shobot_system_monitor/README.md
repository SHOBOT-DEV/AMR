# shobot_system_monitor

Heartbeat / liveness monitor for SHOBOT. Publishes `/system/heartbeat` JSON with CPU/memory stats plus sensor/node aliveness based on configured heartbeat topics.

## Usage
```bash
ros2 launch shobot_system_monitor system_heartbeat_launch.py \
  sensor_topics:="[laser:/scan:1.0,imu:/imu/data:1.0]" \
  node_topics:="[nav:/navigation_status:2.0]"
```

- `heartbeat_topic` (default `/system/heartbeat`)
- `rate_hz` (default `1.0`)
- `sensor_topics` / `node_topics`: list of `"name:topic:timeout_sec"` strings to watch. A topic is alive if a message was seen within `timeout_sec`.

Payload example:
```json
{
  "timestamp": 1700000000000000000,
  "cpu_percent": 12.3,
  "mem_percent": 45.6,
  "load_avg": [0.45, 0.50, 0.60],
  "sensors": [{"name": "laser", "topic": "/scan", "alive": true, "age_sec": 0.2, "timeout_sec": 1.0}],
  "nodes": [{"name": "nav", "topic": "/navigation_status", "alive": true, "age_sec": 0.5, "timeout_sec": 2.0}],
  "ok": true
}
```
