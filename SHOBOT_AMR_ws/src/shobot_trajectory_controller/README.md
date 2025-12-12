# shobot_trajectory_controller

Smooths incoming `cmd_vel` and enforces acceleration limits before sending to motors.

## Usage
- Launched in `shobot_navigation/shobot_robot_bringup.py`.

Key params:
- `input_topic`: source velocity (e.g., mux output)
- `output_topic`: filtered velocity
- `rate_hz`: update rate
- `max_linear_accel`, `max_angular_accel`: acceleration caps
