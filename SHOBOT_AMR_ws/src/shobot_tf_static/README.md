# shobot_tf_static

Static TF2 broadcaster for SHOBOT. Publishes:
- `base_link -> laser`
- `base_link -> imu`
- `base_link -> camera_frame`
- `map -> odom` (optional; leave off if Nav2 provides this)

## Launch
```bash
ros2 launch shobot_tf_static static_tf_publisher_launch.py
```

Edit `config/static_transforms.yaml` to set translations (m), rotations (radians), and whether to publish `map -> odom`.
