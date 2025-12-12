# shobot_laser_filters

Filters LaserScan data (range clamping/cleaning).

## Launch
- `ros2 launch shobot_laser_filters shobot_laser_filters_launch.py`

Key params:
- `input_topic`, `output_topic`
- `range_min`, `range_max`: clamp ranges
- `replace_invalid_with`: value to use for NaN/inf
