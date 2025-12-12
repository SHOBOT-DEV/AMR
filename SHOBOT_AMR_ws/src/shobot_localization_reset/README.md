# shobot_localization_reset

Publish `/initialpose` when localization needs a manual reset (AMCL divergence, unknown start pose, operator reposition).

Features:
- Service: `reset_localization` (std_srvs/Trigger) publishes a configured default pose.
- Relay: subscribes to `/reset_initialpose` (PoseWithCovarianceStamped) and republishes to `/initialpose`, ensuring `frame_id` is set (default `map`).
- Configurable covariance for x, y, yaw on the default pose.

Launch:
```bash
ros2 launch shobot_localization_reset localization_reset_launch.py \
  default_pose_xy_yaw:="[1.0, 0.0, 1.57]" \
  covariance_x:=0.09 covariance_y:=0.09 covariance_yaw:=0.05
```

Manual trigger:
```bash
ros2 service call /reset_localization std_srvs/srv/Trigger "{}"
```

Relay from an external pose (e.g., RViz 2D Pose Estimate):
- Publish to `/reset_initialpose` (PoseWithCovarianceStamped); the node forwards to `/initialpose`.
