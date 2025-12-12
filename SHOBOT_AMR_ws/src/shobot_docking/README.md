# shobot_docking

Docking action server for aligning with a charging station.

## Launch
- `ros2 launch shobot_docking shobot_docking_launch.py`

Features:
- Provides `dock` action (see `shobot_docking/action/Dock.action`) to start docking.
- Can be triggered by mission control after navigation.

Key params: docking frame/target settings (see launch). Action goal is boolean `start=true`.
