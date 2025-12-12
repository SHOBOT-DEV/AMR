# SHOBOT Workspace (ROS2)

ROS 2 workspace for SHOBOT navigation, safety, perception, teleop, and integration nodes. Use this as the entry point for building and launching the robot stack.

## Workspace layout
- Navigation and planning: `shobot_navigation`, `shobot_navigation_server`, `shobot_local_planner` (DWA, path-based/predetermined navigation), `shobot_trajectory_controller`
- Perception and sensing: `shobot_laser_filters`, `shobot_scan_merger`, `shobot_pointcloud_assembler`, `shobot_pointcloud_filter`, `shobot_pointcloud_to_laserscan`, `shobot_yolo_detection`, IMU, wheel encoder
- Localization and mapping: `shobot_robot_localization` (base odom layer + IMU layer + visual SLAM layer + LiDAR layer + camera layer), `shobot_robot_pose_publisher`, `shobot_localization_reset`
- TF and frames: `shobot_tf_static` (base_link→laser/imu/camera_frame, optional map→odom)
- Costmap and safety: `shobot_costmap_plugins`, `shobot_costmap_safety_layer`, `shobot_zone_management`
- Mission control: `shobot_mission_control`, `shobot_mission_handler`
- State management: `shobot_fsm` (central robot state machine)
- Specialized features: `shobot_docking`, `shobot_dock_detection`, `shobot_teleop`, `shobot_twist_mux`, `shobot_status_aggregator`
- Health/monitoring: `shobot_system_monitor` (heartbeat), `shobot_sensors` battery monitor
- Communication and interface: `shobot_api_bridge`, `shobot_rosbridge_suite`, `web_video_server`

## Build
```bash
colcon build --symlink-install
source install/setup.bash
```
Build is required so generated actions/services (dock, navigate, etc.) exist for launch files.

## Quick bring-up (teleop + nav + safety)
Minimal stack (teleop, mux, smoothing, safety, nav client, YOLO optional):
```bash
ros2 launch shobot_navigation shobot_robot_bringup.py
```
Switch planner mode (Nav2 DWA vs predetermined path -> final pose):
```bash
ros2 launch shobot_navigation shobot_robot_bringup.py planner_mode:=path path_goal_topic:=/plan_goal
```
In `planner_mode:=path`, a `nav_msgs/Path` on `path_goal_topic` is executed sequentially (each pose forwarded to Nav2). Adjust parameters and remaps in the launch file to match your topics and frame names.

## Common launches
- `shobot_navigation/shobot_navigation_launch.py`: Nav2 client.
- `shobot_navigation_server/shobot_navigation_server_launch.py`: Nav2 service wrapper.
- `shobot_docking/shobot_docking_launch.py`: Dock action server.
- `shobot_dock_detection/dock_detection_launch.py`: Fuse AprilTag/QR/ArUco/magnetic/colored markers into `/dock_pose` + `/dock_detected`.
- `shobot_local_planner/shobot_local_planner_launch.py`: Waypoint follower.
- `shobot_yolo_detection/shobot_yolo_detection_launch.py`: YOLOv10 detector.
- `shobot_costmap_plugins/shobot_costmap_plugins_launch.py`: PointCloud to OccupancyGrid.
- `shobot_costmap_safety_layer/shobot_costmap_safety_layer_launch.py`: Safety stop from scans.
- `shobot_laser_filters/shobot_laser_filters_launch.py`: LaserScan filtering.
- `shobot_tf_static/static_tf_publisher_launch.py`: Static TFs (base_link→laser/imu/camera_frame; optional map→odom).
- `shobot_system_monitor/system_heartbeat_launch.py`: CPU/mem + topic liveness heartbeat on `/system/heartbeat`.
- `shobot_sensors/shobot_sensors_launch.py`: IMU republisher, wheel encoder odometry, battery monitor.
- `shobot_robot_localization/ekf_launch.py`: EKF fusing /odom, /odom/wheel, /imu/data.
- `shobot_localization_reset/localization_reset_launch.py`: Publish `/initialpose` via Trigger service or relay.
- `shobot_mission_control/shobot_mission_control_launch.py`: Single-task input or batched missions.

## TF tree
- Static transforms: `ros2 launch shobot_tf_static static_tf_publisher_launch.py` (edit `config/static_transforms.yaml` for base_link→laser/imu/camera_frame and optional map→odom).

## Sensors (IMU + wheel encoder + battery)
- Republish IMU with consistent frame/covariance and compute wheel odometry:
```bash
ros2 launch shobot_sensors shobot_sensors_launch.py imu_input_topic:=/imu/raw left_encoder_topic:=/left_wheel_encoder right_encoder_topic:=/right_wheel_encoder
```
Tune `wheel_radius`, `wheelbase`, and `ticks_per_rev` to your hardware; odometry publishes on `/odom/wheel`.
- Battery monitor (publishes `/battery_status` and `/battery_low`):
```bash
ros2 launch shobot_sensors shobot_sensors_launch.py battery_topic:=/battery_state battery_low_threshold:=0.2 battery_critical_threshold:=0.1
```

## Docking
- Detection fusion: `ros2 launch shobot_dock_detection dock_detection_launch.py sources:="[apriltag:/dock/apriltag_pose:1.0,aruco:/dock/aruco_pose:1.0]" pose_frame_override:=map` produces `/dock_pose` + `/dock_detected`.
- Dock action server: `ros2 launch shobot_docking shobot_docking_launch.py`.

## Localization reset
- Manual reset of AMCL/initial pose: `ros2 launch shobot_localization_reset localization_reset_launch.py default_pose_xy_yaw:="[x,y,yaw]"`.
- Trigger default pose: `ros2 service call /reset_localization std_srvs/srv/Trigger "{}"`.
- Relay any `PoseWithCovarianceStamped` on `/reset_initialpose` to `/initialpose`.

## Heartbeat / liveness
- `ros2 launch shobot_system_monitor system_heartbeat_launch.py sensor_topics:="[laser:/scan:1.0]" node_topics:="[nav:/navigation_status:2.0]"` publishes `/system/heartbeat` JSON with CPU %, mem %, load, and topic liveness.

## Localization and mapping layers
- Base layer: wheel odometry (`/odom/wheel`) and any fused `/odom` source.
- IMU layer: `/imu/data` republished by `shobot_sensors`.
- Visual SLAM layer: `/rtabmap/odom` (edit if your visual SLAM differs).
- LiDAR layer: `/lidar_odom` (e.g., scan matching or lidar odom).
- Camera layer: `/vo` (monocular/stereo visual odometry).
Update `shobot_robot_localization/config/ekf.yaml` topic names and covariances for your stack before deployment.

## Notes
- Install external deps required by specific nodes (examples: `ultralytics`, `pyrealsense2`, `cv_bridge`, Nav2).
- Use `ros2 launch ... --show-args` to see configurable parameters for each launch file.
- Tune topics, frames, and controller gains per robot configuration before field deployment.
- Safety landmarks: `shobot_costmap_safety_layer` can also listen to a PoseArray `landmark_topic` and stop if any landmark is within `landmark_stop_radius`.
- Mission control: publish single `PoseStamped` tasks to `/task_goal` (one at a time) or batch missions as JSON to `/mission_queue`. Only one mission/task executes at a time; incoming batches queue.
