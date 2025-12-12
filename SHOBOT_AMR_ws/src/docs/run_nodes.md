# SHOBOT Runbook (ROS 2)

## Build once
```bash
colcon build --symlink-install
source install/setup.bash
```

## Navigation / Planning
- Nav2 action client: `ros2 launch shobot_navigation shobot_navigation_launch.py`
- Nav2 service wrapper: `ros2 launch shobot_navigation_server shobot_navigation_server_launch.py`
- Full bringup (teleop + mux + safety + nav client + optional YOLO):  
  `ros2 launch shobot_navigation shobot_robot_bringup.py`
- Local planner / waypoint follower: `ros2 launch shobot_local_planner shobot_local_planner_launch.py`

## Docking
- Dock action server: `ros2 launch shobot_docking shobot_docking_launch.py`
- Dock detection fusion (AprilTag/QR/ArUco/magnetic/colored markers → `/dock_pose`, `/dock_detected`):  
  `ros2 launch shobot_dock_detection dock_detection_launch.py sources:="[apriltag:/dock/apriltag_pose:1.0,aruco:/dock/aruco_pose:1.0]" pose_frame_override:=map`

## Localization
- EKF: `ros2 launch shobot_robot_localization ekf_launch.py`
- Localization reset (/initialpose):  
  `ros2 launch shobot_localization_reset localization_reset_launch.py default_pose_xy_yaw:="[x,y,yaw]"`  
  Trigger default pose: `ros2 service call /reset_localization std_srvs/srv/Trigger "{}"`  
  Relay pose: publish `PoseWithCovarianceStamped` to `/reset_initialpose`.

## TF / Frames
- Static transforms (base_link→laser/imu/camera_frame; optional map→odom):  
  `ros2 launch shobot_tf_static static_tf_publisher_launch.py`  
  Edit offsets in `shobot_tf_static/config/static_transforms.yaml`.

## Sensors
- IMU repub + wheel odom + battery:  
  `ros2 launch shobot_sensors shobot_sensors_launch.py imu_input_topic:=/imu/raw left_encoder_topic:=/left_wheel_encoder right_encoder_topic:=/right_wheel_encoder battery_topic:=/battery_state battery_low_threshold:=0.2 battery_critical_threshold:=0.1`
- Laser filters: `ros2 launch shobot_laser_filters shobot_laser_filters_launch.py`
- PointCloud → costmap tools: `ros2 launch shobot_costmap_plugins shobot_costmap_plugins_launch.py`
- Costmap safety stop: `ros2 launch shobot_costmap_safety_layer shobot_costmap_safety_layer_launch.py`

## Teleop / Command routing
- Teleop keyboard: `ros2 run shobot_teleop teleop_key`
- Twist mux: `ros2 launch shobot_twist_mux shobot_twist_mux_launch.py`
- Trajectory smoothing: `ros2 launch shobot_trajectory_controller shobot_trajectory_controller_launch.py`

## Perception
- YOLOv10 detector: `ros2 launch shobot_yolo_detection shobot_yolo_detection_launch.py`
- PointCloud filters/assembler: see package launch files (`shobot_pointcloud_filter`, `shobot_pointcloud_assembler`, `shobot_pointcloud_to_laserscan`, `shobot_scan_merger`).

## Missions / State
- Mission control: `ros2 launch shobot_mission_control shobot_mission_control_launch.py`
- Mission handler: `ros2 launch shobot_mission_handler shobot_mission_handler_launch.py`
- FSM: `ros2 launch shobot_fsm shobot_fsm_launch.py`

## Monitoring / Status
- Heartbeat (CPU/mem + topic liveness → `/system/heartbeat`):  
  `ros2 launch shobot_system_monitor system_heartbeat_launch.py sensor_topics:="[laser:/scan:1.0]" node_topics:="[nav:/navigation_status:2.0]"`
- Status aggregator: `ros2 launch shobot_status_aggregator shobot_status_aggregator_launch.py`
- Battery monitor is part of `shobot_sensors_launch.py` (publishes `/battery_status`, `/battery_low`).

## API / Interfaces
- API bridge: `ros2 launch shobot_api_bridge api_bridge_launch.py`
- ROSBridge: `ros2 launch shobot_rosbridge_suite shobot_rosbridge_suite_launch.py`
- Web video server: see `web_video_server` (launch/config as needed).
