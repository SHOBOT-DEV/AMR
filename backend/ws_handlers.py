from __future__ import annotations

from flask_socketio import emit

from backend.API.V1 import frontend_store
from backend.socketio import socketio


@socketio.on("connect", namespace="/stream/robot_pose")
def handle_robot_pose_connect():
    snapshot = frontend_store.get_ros2_snapshot()
    emit("robot_pose", snapshot.get("pose", {}))


@socketio.on("connect", namespace="/stream/costmap")
def handle_costmap_connect():
    costmap = frontend_store.get_ros2_costmap()
    emit("costmap", costmap or {})


@socketio.on("connect", namespace="/stream/logs")
def handle_logs_connect():
    emit("logs", frontend_store.get_logs())
