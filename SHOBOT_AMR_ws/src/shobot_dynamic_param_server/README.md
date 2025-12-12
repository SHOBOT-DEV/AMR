# shobot_dynamic_param_server

Runtime parameter server for live tuning of common navigation settings. Supports direct runtime changes (`ros2 param set`) and optional propagation to other nodes.

## Tunable parameters
- `speed_limit_linear` / `speed_limit_angular`
- `safety_stop_distance` / `safety_slowdown_distance`
- `costmap_inflation_radius` / `costmap_inscribed_radius`
- `laser_min_range` / `laser_max_range`
- `nav2_replan_dist_threshold` / `nav2_replan_time_threshold`
- `propagate_targets`: list of node names to push changes to (same parameter names).

## Launch
```
ros2 launch shobot_dynamic_param_server dynamic_param_server_launch.py \
  propagate_targets:="[/controller_server,/planner_server]"
```

## Using
- Inspect: `ros2 param list /shobot_dynamic_param_server`
- Get current: `ros2 param get /shobot_dynamic_param_server speed_limit_linear`
- Set live: `ros2 param set /shobot_dynamic_param_server speed_limit_linear 0.8`
- Propagation (optional): add node names to `propagate_targets` so changes are also applied to those nodes via SetParameters.

Notes:
- Values are validated against min/max ranges; invalid updates are rejected with a reason.
- Propagation is best-effort and logged if a target rejects or cannot be reached.
