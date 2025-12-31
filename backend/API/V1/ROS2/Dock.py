from __future__ import annotations

from flask import jsonify, request


def register_ros2_dock_routes(bp, store):
    @bp.route("/ros2/dock/detection", methods=["GET"])
    def ros2_dock_detection():
        dock = store.get_ros2_dock_detection()
        if not dock.get("dock_detected") or dock.get("dock_pose") is None:
            return jsonify({"detected": False})

        pose = dock.get("dock_pose") or {}
        return jsonify(
            {
                "detected": True,
                "x": pose.get("x", 0.0),
                "y": pose.get("y", 0.0),
                "frame_id": pose.get("frame_id", "map"),
            }
        )

    @bp.route("/ros2/dock/detection/status", methods=["GET"])
    def ros2_dock_detection_status():
        dock = store.get_ros2_dock_detection()
        return jsonify(dock.get("dock_status") or {})

    @bp.route("/ros2/dock/start", methods=["POST"])
    def ros2_dock_start():
        status = store.set_ros2_dock_detection(True, status={"status": "accepted"})
        return jsonify({"status": status.get("dock_status", {}).get("status", "accepted")})

    @bp.route("/ros2/dock/action", methods=["GET"])
    def ros2_dock_action_name():
        return jsonify({"name": store.get_ros2_dock_action_name()})

    @bp.route("/ros2/dock/action", methods=["POST"])
    def ros2_set_dock_action_name():
        payload = request.get_json(silent=True) or {}
        name = payload.get("name", "")
        if not isinstance(name, str):
            return jsonify({"success": False, "message": "name must be a string"}), 400
        return jsonify({"success": True, "name": store.set_ros2_dock_action_name(name)})


__all__ = ["register_ros2_dock_routes"]
