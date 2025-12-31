from __future__ import annotations

from flask import jsonify


def register_ros2_safety_routes(bp, store):
    @bp.route("/ros2/safety/status", methods=["GET"])
    def ros2_safety_status():
        return jsonify({"stop": store.get_ros2_safety_stop()})

    @bp.route("/ros2/safety/stop", methods=["POST"])
    def ros2_force_stop():
        store.set_ros2_safety_stop(True)
        return jsonify({"stop": True})


__all__ = ["register_ros2_safety_routes"]
