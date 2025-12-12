# shobot_local_planner

Simple waypoint-following local planner that converts `nav_msgs/Path` into `cmd_vel`. Uses lookahead distance and basic heading control to steer.

## Launch
- `ros2 launch shobot_local_planner shobot_local_planner_launch.py`

Key params:
- `path_topic`: input `nav_msgs/Path`
- `cmd_vel_topic`: output velocity command
- `odom_topic`: robot odometry
- `lookahead_distance`: how far ahead to aim on the path
- `max_linear` / `max_angular`: velocity limits
