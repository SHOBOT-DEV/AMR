# shobot_navigation_server

Wrapper around Nav2 to expose a simple service/action interface for external callers.

## Launch
- `ros2 launch shobot_navigation_server shobot_navigation_server_launch.py`

Features:
- Exposes a service to send navigation goals without dealing with Nav2 actions directly.
- Uses custom `Navigate.srv` defined in `srv/Navigate.srv`.

Key params: service/action names; see launch for overrides. Provide poses in the request to trigger Nav2 navigation.
