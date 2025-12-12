# shobot_log_recorder

Records key ROS topics (sensors, mission events, task failures, error logs) into JSONL files and can play them back for debugging and fleet management.

## Features
- Configurable topic list with per-topic type hints (`topic[:type]`).
- Rotating JSONL files with serialized payloads (base64) plus human-readable text fallback.
- Playback service re-publishes recorded messages at original cadence (scaled by `playback_rate`).

## Usage
```
ros2 launch shobot_log_recorder log_recorder_launch.py \
  output_dir:=~/.shobot/log_recorder \
  sensor_topics:="[/scan:sensor_msgs.msg.LaserScan,/imu/data:sensor_msgs.msg.Imu]" \
  mission_topic:=/mission_status \
  task_failure_topic:=/task_failures \
  error_topic:=/system/errors
```

### Services
- `start_playback` (`std_srvs/Trigger`): Plays back the latest log file, or `playback_file` parameter if set.
- `stop_playback` (`std_srvs/Trigger`): Cancels an active playback session.

### Parameters
- `output_dir` (string): Where to write log files (default `~/.shobot/log_recorder`).
- `max_file_size_mb` (float): Rotate to a new file after this size (default 200 MB).
- `playback_rate` (float): Scale playback timing (1.0 = real-time).
- `sensor_topics` (list): Entries like `topic[:type]` (type optional but recommended for playback).
- `mission_topic`, `task_failure_topic`, `error_topic`: Convenience topics (default `std_msgs/msg/String`).
- `playback_file` (string): Absolute or `~` path to a specific log file to replay (optional).

### Notes
- If a topic type is unknown, the node falls back to `AnyMsg`; messages are stored as text and are **not** replayable.
- Log format: one JSON object per line with `stamp` (ns), `topic`, `type`, and `data_b64` or `text`.
