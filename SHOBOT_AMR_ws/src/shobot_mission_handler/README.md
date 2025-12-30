# shobot_mission_handler

Mission command normalization and validation node for SHOBOT AMR.

`shobot_mission_handler` receives high-level mission commands (JSON), validates and normalizes them into a standardized mission queue, and forwards them to the mission control layer. It also exposes a service to clear active missions.

## Role in the system
This node does not execute navigation or docking directly. It acts as a gatekeeper between external mission sources (UI, web app, FSM, API) and the internal mission execution stack.

```
UI / Web / FSM
      ↓
/mission_command   (JSON)
      ↓
shobot_mission_handler
      ↓
/mission_queue
      ↓
shobot_mission_control → navigation / docking / actions
```

## Inputs
**Topics**
- `/mission_command` (`std_msgs/String`): JSON-encoded mission commands.

**Services**
- `/clear_missions` (`std_srvs/Trigger`): clears all queued missions immediately.

## Outputs
**Topics**
- `/mission_queue` (`std_msgs/String`): normalized mission list for mission control.
- `/mission_handler_status` (`std_msgs/String`): human-readable status updates for UI/logging.

## Accepted mission formats
**Full mission list**
```json
{
  "missions": [
    { "pose": { "x": 2.0, "y": 1.5 }, "dock": false },
    { "pose": { "x": 0.0, "y": 0.0 }, "dock": true }
  ]
}
```

**Shorthand (single mission)**
```json
{ "pose": { "x": 3.0, "y": 0.5 }, "dock": true }
```

**Clear existing missions**
```json
{ "clear": true }
```

## Mission validation rules
- Each mission must contain a `pose` object.
- `pose` must include at least `x` and `y` coordinates.
- Optional `dock` flag must be boolean.
- Invalid or malformed missions are rejected with a clear status message.

## Parameters
- `command_topic` (string, default `/mission_command`): incoming mission command topic.
- `queue_topic` (string, default `/mission_queue`): output mission queue topic.
- `status_topic` (string, default `/mission_handler_status`): status/log topic.

## Usage
Run as part of robot bring-up or mission pipeline:
```bash
ros2 run shobot_mission_handler shobot_mission_handler
```

Or via launch file:
```bash
ros2 launch shobot_mission_handler shobot_mission_handler_launch.py
```
