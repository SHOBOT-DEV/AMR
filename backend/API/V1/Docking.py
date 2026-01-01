from __future__ import annotations

from flask import jsonify


def register_docking_routes(bp, store):
    @bp.route("/docking/detection", methods=["GET"])
    def docking_detection():
        dock = store.get_ros2_dock_detection()
        return jsonify({"success": True, "data": dock})

    @bp.route("/docking/status", methods=["GET"])
    def docking_status():
        dock = store.get_ros2_dock_detection()
        return jsonify({"success": True, "status": dock.get("dock_status", {})})

    @bp.route("/docking/start", methods=["POST"])
    def docking_start():
        dock = store.set_ros2_dock_detection(True, status={"status": "accepted"})
        return jsonify({"success": True, "data": dock})

    @bp.route("/docking/cancel", methods=["POST"])
    def docking_cancel():
        dock = store.set_ros2_dock_detection(False, status={"status": "canceled"})
        return jsonify({"success": True, "data": dock})

    @bp.route("/docking/reset", methods=["POST"])
    def docking_reset():
        dock = store.reset_docking()
        return jsonify({"success": True, "data": dock})


__all__ = ["register_docking_routes"]
