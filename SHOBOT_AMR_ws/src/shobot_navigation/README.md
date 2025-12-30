# shobot_navigation

Nav2 `NavigateToPose` client for SHOBOT AMR. Forwards pose goals to Nav2 and can execute an entire `nav_msgs/Path` waypoint-by-waypoint. Designed for simple integration with FSMs, mission handlers, teleop/mux pipelines, and higher-level planners.

## What this node does
- Subscribes to pose goals (`geometry_msgs/PoseStamped`).
- Forwards goals to Nav2’s `NavigateToPose` action.
- Publishes navigation status and feedback.
- Supports goal cancellation.
- Optional path mode: accepts a `nav_msgs/Path`, sends each pose sequentially, advances only after success.
- Thin client wrapper only; Nav2 still runs separately.

## Launch
- `ros2 launch shobot_navigation shobot_navigation_launch.py` — standard Nav2 client
- `ros2 launch shobot_navigation shobot_robot_bringup.py` — teleop + mux + safety + navigation

⚠️ Nav2 must already be running (map server, localization, controller, planner).

## Topics
**Subscribed**
- `goal_topic` (`geometry_msgs/PoseStamped`): single navigation goal.
- `path_goal_topic` (`nav_msgs/Path`): path input when `planner_mode:=path`.

**Published**
- `/navigation_status` (`std_msgs/String`): high-level navigation state.
- `/navigation_feedback` (`std_msgs/String`): throttled feedback (distance, speed).

**Services**
- `/cancel_navigation` (`std_srvs/Trigger`): cancel active navigation goal.

## Key parameters
**Core**
- `goal_topic` (string, default `/goal_pose`): pose goal topic.
- `status_topic` (string, default `/navigation_status`): status output.
- `feedback_topic` (string, default `/navigation_feedback`): feedback output.
- `nav2_action_name` (string, default `/navigate_to_pose`): Nav2 action server.
- `feedback_throttle_sec` (float, default `0.5`): feedback publish interval.

**Planner mode**
- `planner_mode` (enum, default `dwa`): `dwa` or `path`.
- `path_goal_topic` (string, default `/plan_goal`): required when `planner_mode:=path`.

## Modes explained
- `dwa` (default): each incoming pose is sent directly to Nav2.
- `path`: receive a `nav_msgs/Path` and execute sequentially; move to next waypoint only after success.

## Usage examples
**Send a single goal**
```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "
header:
  frame_id: 'map'
pose:
  position: {x: 2.0, y: 1.5, z: 0.0}
  orientation: {w: 1.0}"
```

**Send a path (planner_mode:=path)**
```bash
ros2 topic pub /plan_goal nav_msgs/Path "
header:
  frame_id: 'map'
poses:
- pose:
    position: {x: 1.0, y: 1.0, z: 0.0}
    orientation: {w: 1.0}
- pose:
    position: {x: 2.0, y: 1.0, z: 0.0}
    orientation: {w: 1.0}"
```

**Cancel navigation**
```bash
ros2 service call /cancel_navigation std_srvs/srv/Trigger
```

## Typical architecture
```
Mission Handler / FSM
        |
        |  PoseStamped / Path
        v
shobot_navigation
        |
        |  NavigateToPose action
        v
        Nav2 Stack
```

## Design notes
- Only one active goal at a time; new goals cancel previous ones.
- Path execution is sequential (no smoothing).
- Feedback messages are human-readable.
- No goal queue beyond the current path.

## Recommended use cases
- Mission execution (dock → navigate → inspect).
- FSM-based autonomy.
- Web/UI control panels.
- Research and prototyping.
- Integration with `shobot_navigation_server`.

## Related SHOBOT packages
- `shobot_navigation_server` — service-based Nav2 wrapper.
- `shobot_twist_mux` — velocity arbitration.
- `shobot_trajectory_controller` — acceleration limiting.
- `shobot_mission_handler` — task orchestration.
- `nav2_bringup` — core Nav2 stack.
