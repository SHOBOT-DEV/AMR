# shobot_teleop

Teleoperation input node (e.g., keyboard) publishing `cmd_vel/teleop`.

## Usage
Launch as part of bring-up or standalone.

Keys: `w/x` increase/decrease forward speed, `a/d` steer left/right, space or `s` stops, CTRL-C exits.

Key params:
- `teleop_topic`: output Twist topic (default `/joy/cmd_vel` or remapped to mux input)
- `model`: velocity limits preset (`burger`, `waffle`, etc.)
- `rate_hz`: publish rate
