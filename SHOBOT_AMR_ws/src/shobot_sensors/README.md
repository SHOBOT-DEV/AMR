# shobot_sensors

IMU republisher and wheel encoder odometry nodes for SHOBOT.

## Launch
- `ros2 launch shobot_sensors shobot_sensors_launch.py`

Features:
- IMU republisher: sets consistent frame_id and optional covariances on IMU messages.
- Wheel odometry: converts encoder ticks to `nav_msgs/Odometry`.
- Battery monitor: republishes battery status and low-battery flag.

Key params:
- IMU: `input_topic`, `output_topic`, `frame_id`, optional covariance arrays.
- Encoders: `left_encoder_topic`, `right_encoder_topic`, `wheel_radius`, `wheelbase`, `ticks_per_rev`, `odom_topic`, `odom_frame`, `base_frame`.
- Battery: `battery_topic`, `battery_status_topic`, `battery_low_topic`, `battery_low_threshold`, `battery_critical_threshold`.
