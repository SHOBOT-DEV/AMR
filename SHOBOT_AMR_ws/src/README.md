# SHOBOT Workspace (ROS2)

This workspace contains SHOBOT ROS2 nodes for teleop, navigation, safety, perception, and utilities. Key packages and entry points:

- **shobot_teleop**: Keyboard teleoperation publishing `cmd_vel/teleop`.
- **shobot_twist_mux**: Priority mux (`safety` > `teleop` > `nav`) to produce `cmd_vel_raw`.
- **shobot_trajectory_controller**: Acceleration-limited smoothing from `cmd_vel_raw` to `cmd_vel`.
- **shobot_costmap_safety_layer**: Generates `safety_stop` from LaserScan thresholds.
- **shobot_laser_filters**: Range clamp / NaN handling for LaserScan.
- **shobot_navigation**: Nav2 NavigateToPose client (goal subscriber, status/feedback publishers).
- **shobot_navigation_server**: Service wrapper to send Nav2 goals.
- **shobot_local_planner**: Simple waypoint follower publishing `cmd_vel`.
- **shobot_mission_control / shobot_mission_handler**: Mission queue execution and command normalization; optional docking via `shobot_docking`.
- **shobot_docking**: Minimal docking action server using odom and cmd_vel.
- **shobot_yolo_detection**: YOLOv10 detections with RealSense or ROS image topics; publishes JSON + annotated images.
- **shobot_pointcloud_assembler / shobot_pointcloud_to_laserscan**: Point cloud utilities (merge clouds, project to LaserScan).
- **shobot_costmap_plugins**: PointCloud-to-OccupancyGrid builder for costmap layering.
- **shobot_status_aggregator**: Combines status topics into a JSON summary.
- **shobot_rosbridge_suite**: Announces rosbridge endpoint info.

## Quick bring-up (teleop + nav + safety)
Launch a minimal stack (teleop, mux, smoothing, safety, nav client, YOLO optional):
```bash
ros2 launch shobot_navigation shobot_robot_bringup.py
```
Adjust params/remaps in the launch file to match your topics.

## Common launches
- `shobot_navigation/shobot_navigation_launch.py`: Nav2 client.
- `shobot_navigation_server/shobot_navigation_server_launch.py`: Nav2 service wrapper.
- `shobot_docking/shobot_docking_launch.py`: Dock action server.
- `shobot_local_planner/shobot_local_planner_launch.py`: Waypoint follower.
- `shobot_yolo_detection/shobot_yolo_detection_launch.py`: YOLOv10 detector.
- `shobot_costmap_plugins/shobot_costmap_plugins_launch.py`: PointCloud → OccupancyGrid.
- `shobot_costmap_safety_layer/shobot_costmap_safety_layer_launch.py`: Safety stop from scans.
- `shobot_laser_filters/shobot_laser_filters_launch.py`: LaserScan filtering.

## Notes
- Ensure dependencies (e.g., `ultralytics`, `pyrealsense2`, `cv_bridge`, Nav2) are installed in your environment.
- Build the workspace with `colcon build` so custom actions/services (Dock, Navigate) are generated.
- Tune topics/parameters in each launch file to fit your robot’s configuration.
