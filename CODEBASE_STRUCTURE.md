# SHOBOT AMR Codebase Structure

## Executive Summary

This repository contains a ROS 2 (colcon) workspace for the SHOBOT AMR stack, a
React-based frontend, and a Flask backend API. The ROS 2 workspace holds the
robot nodes and launch files; the web UI and API live in separate top-level
folders.

## Top-Level Layout

```
AMR/
├── backend/             # Flask API server (SQLite via SQLAlchemy)
├── FrontEnd/            # React web UI (Create React App)
├── SHOBOT_AMR_ws/       # ROS 2 colcon workspace
├── UI/                  # Additional UI sources
├── docs/                # Documentation
├── RL/                  # Reinforcement-learning related work
├── API_ENDPOINTS.md     # API documentation
├── README.MD            # Repo README
└── run.sh               # Convenience script
```

## Frontend (React)

**Location**: `FrontEnd/`

- React application (Create React App)
- `package.json` defines the web dependencies and scripts
- Proxy configured to `http://localhost:5000` for API calls

## Backend (Flask)

**Location**: `backend/`

- Flask API server
- SQLAlchemy with SQLite database stored in `backend/instance/users.db`
- Routes and API logic live in `backend/routes/` and `backend/API/`

## ROS 2 Workspace

**Location**: `SHOBOT_AMR_ws/`

- ROS 2 colcon workspace
- Build artifacts live in `build/`, `install/`, and `log/`
- Packages are under `SHOBOT_AMR_ws/src/`

### ROS 2 Packages (by directory)

```
SHOBOT_AMR_ws/src/
├── docs
├── ROS package
├── shobot_api_bridge
├── shobot_bringup
├── shobot_costmap_plugins
├── shobot_costmap_safety_layer
├── shobot_dock_detection
├── shobot_docking
├── shobot_dynamic_param_server
├── shobot_fsm
├── shobot_laser_filters
├── shobot_localization_reset
├── shobot_local_planner
├── shobot_log_recorder
├── shobot_mission_control
├── shobot_mission_handler
├── shobot_navigation
├── shobot_navigation_server
├── shobot_pointcloud_assembler
├── shobot_pointcloud_filter
├── shobot_pointcloud_to_laserscan
├── shobot_recovery
├── shobot_rl_agent
├── shobot_robot_diagnostics
├── shobot_robot_localization
├── shobot_robot_pose_publisher
├── shobot_rosbridge_suite
├── shobot_scan_merger
├── shobot_sensors
├── shobot_status_aggregator
├── shobot_system_monitor
├── shobot_teleop
├── shobot_tf_static
├── shobot_trajectory_controller
├── shobot_twist_mux
├── shobot_yolo_detection
├── shobot_zone_management
└── web_video_server
```

## Build System

- **ROS 2**: `colcon build --symlink-install`
- **Frontend**: `npm start` / `npm build`
- **Backend**: `python backend/run.py` (or equivalent entrypoint)

## Additional Notes

- See `SHOBOT_AMR_ws/README.md` for ROS 2 package details and launch examples.
- See `API_ENDPOINTS.md` for backend API usage.
