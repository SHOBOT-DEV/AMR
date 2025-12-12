# ROS package (scripts)

Utility scripts for motor control and mapping helpers.

## Scripts
- `motor_controll.py`: motor control with encoder publishing and safety logic; listen on `/cmd_vel`, `/scan`, `/detection_info`. Adjust speed scale by publishing String `'i'` (increase) or `'d'` (decrease) to `/motor_speed_scale`.
- `teleop_motor.py`: teleop motor control.
- `depth.py`: depth processing helper.
- `map_combiner.py`: combines maps/point clouds (usage depends on script args).
