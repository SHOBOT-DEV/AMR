# shobot_navigation

Nav2 NavigateToPose client for SHOBOT. Listens for goals and forwards them to Nav2; can also consume a `nav_msgs/Path` and execute each waypoint in order (predetermined navigation).

## Launch
- `ros2 launch shobot_navigation shobot_navigation_launch.py` (Nav2 client)
- `ros2 launch shobot_navigation shobot_robot_bringup.py` (teleop + mux + safety + nav)

Key params (set in launch or via CLI):
- `goal_topic` (PoseStamped goals)
- `nav2_action_name` (Nav2 action name, default `navigate_to_pose`)
- `planner_mode`: `dwa` (Nav2 default) or `path` (execute Path sequentially)
- `path_goal_topic`: `nav_msgs/Path` topic when using `planner_mode:=path`
