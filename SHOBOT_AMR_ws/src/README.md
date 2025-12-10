# SHOBOT Workspace (ROS2)

ROS 2 workspace for SHOBOT navigation, safety, perception, teleop, and integration nodes. Use this as the entry point for building and launching the robot stack.

## Workspace layout
- Navigation and planning: `shobot_navigation`, `shobot_navigation_server`, `shobot_local_planner` (DWA, path-based/predetermined navigation), `shobot_trajectory_controller`
- Perception and sensing: `shobot_laser_filters`, `shobot_scan_merger`, `shobot_pointcloud_assembler`, `shobot_pointcloud_filter`, `shobot_pointcloud_to_laserscan`, `shobot_yolo_detection`, IMU, wheel encoder
- Localization and mapping: `shobot_robot_localization`, `shobot_robot_pose_publisher`
- Costmap and safety: `shobot_costmap_plugins`, `shobot_costmap_safety_layer`, `shobot_zone_management`
- Mission control: `shobot_mission_control`, `shobot_mission_handler`
- Specialized features: `shobot_docking`, `shobot_teleop`, `shobot_twist_mux`, `shobot_status_aggregator`
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
- `shobot_local_planner/shobot_local_planner_launch.py`: Waypoint follower.
- `shobot_yolo_detection/shobot_yolo_detection_launch.py`: YOLOv10 detector.
- `shobot_costmap_plugins/shobot_costmap_plugins_launch.py`: PointCloud to OccupancyGrid.
- `shobot_costmap_safety_layer/shobot_costmap_safety_layer_launch.py`: Safety stop from scans.
- `shobot_laser_filters/shobot_laser_filters_launch.py`: LaserScan filtering.

## Notes
- Install external deps required by specific nodes (examples: `ultralytics`, `pyrealsense2`, `cv_bridge`, Nav2).
- Use `ros2 launch ... --show-args` to see configurable parameters for each launch file.
- Tune topics, frames, and controller gains per robot configuration before field deployment.
