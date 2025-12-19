# shobot_rl_agent (ROS 2 Jazzy)

Lightweight RL inference wrapper for SHOBOT. Subscribes to LaserScan + Odometry and publishes Twist actions. If a TorchScript policy is provided, it is used; otherwise a simple heuristic keeps the robot moving while avoiding close obstacles.

## Build
```bash
cd SHOBOT_AMR_ws
colcon build --symlink-install --packages-select shobot_rl_agent
source install/setup.bash
```

## Run
```bash
ros2 launch shobot_rl_agent shobot_rl_agent_launch.py \
  model_path:=/absolute/path/to/policy.pt \
  scan_topic:=/scan \
  odom_topic:=/odom \
  cmd_vel_topic:=/cmd_vel
```

`model_path` may be empty to use the heuristic fallback. The node also exposes optional parameters:
- `linear_speed` (default 0.2 m/s)
- `angular_speed` (default 0.4 rad/s)

## Policy notes
- Expects a TorchScript model that takes a single batch tensor shaped `[1, N]` where `N = len(scan.ranges) + 2` (`odom.twist.linear.x`, `odom.twist.angular.z`) and returns `[1, 2]` for `(linear_x, angular_z)`.
- If Torch is unavailable or the model fails to load, the node logs a warning and continues with the heuristic policy.

---

# RL Sensor Fusion + Intent Prediction Node
- Human-aware intent prediction with sensor fusion.
- Inputs: camera bounding boxes (`vision_msgs/Detection2DArray`), LiDAR (`LaserScan`), IMU (`Imu`), depth motion (`Image`).
- Outputs: decision string (`slow_down|reroute|yield|proceed`) and a suggested `Twist`.

Launch:
```bash
ros2 launch shobot_rl_agent shobot_rl_intent_launch.py \
  model_path:=/absolute/path/to/intent_policy.pt \
  bbox_topic:=/detections \
  scan_topic:=/scan \
  imu_topic:=/imu \
  depth_topic:=/depth/image_raw \
  decision_topic:=/rl_intent/decision \
  cmd_vel_topic:=/rl_intent/cmd_vel
```

Policy shape (expected):
- TorchScript model taking `[1, 4]` → logits `[1, 4]` for classes in order `proceed, slow_down, yield, reroute`.
- Features: `[min_range, human_count, imu_wz, depth_available_flag]`.

---

# Dynamic Speed Modulation Node
- Safety/efficiency speed controller (steering untouched).
- Inputs: obstacle density (`LaserScan`), curvature ahead (`nav_msgs/Path`), floor conditions (`std_msgs/Float32`), human proximity (`std_msgs/Float32`), incoming velocity (`Twist`).
- Outputs: modulated `Twist` and a speed multiplier (`/rl_speed/multiplier`).

Launch:
```bash
ros2 launch shobot_rl_agent shobot_rl_speed_launch.py \
  model_path:=/absolute/path/to/speed_policy.pt \
  scan_topic:=/scan \
  path_topic:=/plan \
  floor_topic:=/floor_friction \
  proximity_topic:=/humans/proximity \
  input_cmd_topic:=/cmd_vel_raw \
  output_cmd_topic:=/cmd_vel_modulated
```

Policy shape (expected):
- TorchScript model taking `[1, 5]` → scalar multiplier `[1]`.
- Features: `[min_range, avg_curvature, floor_coeff, human_proximity, commanded_speed]`.
