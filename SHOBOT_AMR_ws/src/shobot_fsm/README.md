# SHOBOT Finite State Machine

ROS 2 node that centralizes robot state transitions so behavior stays predictable.

## States
- `IDLE`, `MOVING`, `BLOCKED`, `DOCKING`, `CHARGING`, `ERROR`, `RECOVERY`

## Interfaces
- Subscribes: `/robot_state_request` (`std_msgs/String`) — JSON `{ "state": "MOVING", "reason": "nav goal", "force": false }` or raw state string.
- Publishes: `/robot_state` (`std_msgs/String`) — JSON describing current state, previous state, timestamp.
- Publishes: `/fsm_status` (`std_msgs/String`) — human-readable status log.

Set `initial_state`, topics, and heartbeat rate via parameters.
