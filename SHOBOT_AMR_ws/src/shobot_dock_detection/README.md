# shobot_dock_detection

Aggregates multiple dock detectors (AprilTag / QR / ArUco / magnetic / colored markers) into unified topics:
- `/dock_pose` (PoseStamped)
- `/dock_detected` (Bool)

Supports any upstream detector that outputs `PoseStamped`. Configure sources as `"name:topic:timeout_sec"`; the first fresh source (list order) wins.

Launch example:
```bash
ros2 launch shobot_dock_detection dock_detection_launch.py \
  sources:="[apriltag:/dock/apriltag_pose:1.0,aruco:/dock/aruco_pose:1.0]" \
  pose_frame_override:=map
```

Status/debug:
- Optional JSON status on `/dock_detection_status` (enable via `publish_status:=true`).
- Parameters: `dock_pose_topic`, `dock_detected_topic`, `pose_frame_override`.

Notes:
- Ensure each detector publishes `PoseStamped` (frame_id set to its estimate). Set `pose_frame_override` if you want all outputs in a fixed frame.
- Timeouts decide if a detection is considered alive; stale detections are ignored.
