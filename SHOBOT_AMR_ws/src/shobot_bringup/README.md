# shobot_bringup

One-command ROS 2 bringup for SHOBOT. Aggregates key launch files (tf static, sensors, filters, safety layer, twist mux, navigation) with optional API bridge and RL nodes.

## Run
```bash
cd SHOBOT_AMR_ws
colcon build --symlink-install --packages-select shobot_bringup
source install/setup.bash
ros2 launch shobot_bringup shobot_full_bringup.launch.py \
  use_api_bridge:=true \
  use_nav_server:=true \
  use_rl:=false
```

Flip `use_rl` to `true` to start the RL intent, speed modulation, and base RL policy nodes (requires `shobot_rl_agent` built and Torch models if desired).

## Notes
- This does not introduce a ROS 1–style master; it simply orchestrates all ROS 2 nodes via launch.
- Adjust underlying package parameters by editing the included launch files or passing their arguments through here as needed.
