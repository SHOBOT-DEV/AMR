# shobot_costmap_safety_layer

Safety layer that stops the robot when obstacles or landmarks are too close.

## Launch
- `ros2 launch shobot_costmap_safety_layer shobot_costmap_safety_layer_launch.py`

Features:
- Monitors LaserScan for obstacles within a threshold.
- Optional PoseArray landmark input to enforce stop zones.

Key params:
- `scan_topic`: incoming LaserScan
- `threshold`: stop if any range below this (meters)
- `safety_stop_topic`: Bool stop flag
- `landmark_topic`: optional PoseArray of landmarks
- `landmark_stop_radius`: stop if any landmark is within this radius
