from __future__ import annotations

from flask import jsonify, request


def register_ros2_navigation_routes(bp, store):
    @bp.route("/navigation/goal", methods=["POST"])
    def navigation_goal():
        payload = request.get_json(silent=True) or {}
        nav = store.set_navigation_goal(payload)
        return jsonify({"success": True, "navigation": nav})

    @bp.route("/navigation/cancel", methods=["POST"])
    def navigation_cancel():
        nav = store.set_navigation_status("CANCELED")
        return jsonify({"success": True, "navigation": nav})

    @bp.route("/navigation/status", methods=["GET"])
    def navigation_status():
        nav = store.get_navigation()
        return jsonify({"status": nav.get("status", "IDLE"), "paused": nav.get("paused", False)})

    @bp.route("/navigation/feedback", methods=["GET"])
    def navigation_feedback():
        nav = store.get_navigation()
        return jsonify(nav.get("feedback", {}))

    @bp.route("/navigation/pause", methods=["POST"])
    def navigation_pause():
        nav = store.set_navigation_paused(True)
        nav = store.set_navigation_status("PAUSED")
        return jsonify({"success": True, "navigation": nav})

    @bp.route("/navigation/resume", methods=["POST"])
    def navigation_resume():
        nav = store.set_navigation_paused(False)
        nav = store.set_navigation_status("ACTIVE")
        return jsonify({"success": True, "navigation": nav})

    @bp.route("/navigation/replan", methods=["POST"])
    def navigation_replan():
        nav = store.set_navigation_status("REPLAN")
        return jsonify({"success": True, "navigation": nav})

    @bp.route("/navigation/path", methods=["GET"])
    def navigation_path():
        nav = store.get_navigation()
        return jsonify(nav.get("path", []))


__all__ = ["register_ros2_navigation_routes"]
