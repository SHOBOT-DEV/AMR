# shobot_mission_control

Manages mission queues and single tasks, executing one at a time via Nav2 and optional docking.

## Launch
- `ros2 launch shobot_mission_control shobot_mission_control_launch.py`

Features:
- Accepts single `PoseStamped` tasks on `/task_goal`.
- Accepts batched JSON missions on `/mission_queue` (each mission = pose + optional dock flag).
- Executes missions one at a time; queues additional requests.
- Triggers optional docking after navigation.

Topics/params:
- `mission_topic`, `task_topic`, `status_topic`
- `nav2_action_name`, `dock_action_name`
Status published on `/mission_status`.
