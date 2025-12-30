# shobot_rl_agent (ROS 2 Jazzy)

Reinforcement Learning–enabled navigation stack for **SHOBOT AMR**.  
This package provides **lightweight RL inference nodes** that can be used independently or combined to enhance navigation in **dynamic, human-populated environments**.

The stack is designed to be:
- **Safe by default** (heuristic fallback if ML fails)
- **Modular** (each node can run standalone)
- **Nav2-compatible**
- **Real-time friendly**

---

## Package Contents

This package contains **three ROS 2 nodes**:

1. **RL Local Planner (`shobot_rl_agent`)**  
   RL-based collision avoidance and smooth local motion control.

2. **RL Intent Prediction (`shobot_rl_intent`)**  
   Human-aware, sensor-fusion–based intent decision making.

3. **Dynamic Speed Modulation (`shobot_rl_speed`)**  
   Safety- and efficiency-aware speed scaling (steering untouched).

---

## System Architecture

```text
                ┌──────────────────┐
 LaserScan ───▶ │  RL Agent Node   │ ───▶ /cmd_vel_raw
 Odometry ───▶  │ (Local Planner)  │
                └──────────────────┘
                           │
                           ▼
                ┌──────────────────┐
                │ Speed Modulation │ ───▶ /cmd_vel
                └──────────────────┘
                           ▲
                           │
        ┌─────────────────────────────────┐
        │      Intent Prediction Node     │
        │ (slow_down / yield / reroute)   │
        └─────────────────────────────────┘
```

## launch & Run
```bash
    ros2 launch shobot_rl_agent shobot_rl_agent_launch.py \
      model_path:=/absolute/path/to/policy.pt \
      scan_topic:=/scan \
      odom_topic:=/odom \
      cmd_vel_topic:=/cmd_vel

```