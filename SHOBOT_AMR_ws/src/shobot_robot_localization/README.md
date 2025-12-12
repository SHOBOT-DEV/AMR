# shobot_robot_localization

EKF-based fusion of wheel odometry, IMU, visual SLAM, LiDAR odom, and camera VO.

## Launch
- `ros2 launch shobot_robot_localization ekf_launch.py`

Key params:
- Topic inputs set in `config/ekf.yaml` (odom, wheel odom, visual SLAM `/rtabmap/odom`, LiDAR odom `/lidar_odom`, camera VO `/vo`, IMU `/imu/data`)
- Override topics via launch args in `ekf_launch.py`
- Frames: `map_frame`, `odom_frame`, `base_link_frame`

## Monitor
- `localization_monitor_node`: checks configured input topics for freshness and publishes `/localization/status` (String) and `/localization/healthy` (Bool). Tune via parameters:
  - `input_topics` (list)
  - `stale_timeout_sec`
  - `check_period_sec`
  - `status_topic`, `healthy_topic`
