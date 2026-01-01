from __future__ import annotations

from flask import jsonify, request


def register_log_bag_routes(bp, store):
    @bp.route("/logs", methods=["GET"])
    def list_logs():
        return jsonify({"success": True, "items": store.get_logs()})

    @bp.route("/logs/<mission_id>", methods=["GET"])
    def get_mission_logs(mission_id: str):
        items = store.get_mission_logs_by_id(mission_id)
        if not items:
            return jsonify({"success": False, "message": "Mission logs not found"}), 404
        return jsonify({"success": True, "items": items})

    @bp.route("/logs/record/start", methods=["POST"])
    def start_log_recording():
        payload = request.get_json(silent=True) or {}
        status = store.start_log_recording(payload.get("source"))
        return jsonify({"success": True, "recording": status})

    @bp.route("/logs/record/stop", methods=["POST"])
    def stop_log_recording():
        status = store.stop_log_recording()
        return jsonify({"success": True, "recording": status})

    @bp.route("/bags", methods=["GET"])
    def list_bags():
        return jsonify({"success": True, "items": store.get_robot_bags()})

    @bp.route("/bags/start", methods=["POST"])
    def start_bag_recording():
        payload = request.get_json(silent=True) or {}
        status = store.start_bag_recording(payload.get("name"))
        return jsonify({"success": True, "recording": status})

    @bp.route("/bags/stop", methods=["POST"])
    def stop_bag_recording():
        item = store.stop_bag_recording()
        return jsonify({"success": True, "bag": item})


__all__ = ["register_log_bag_routes"]
