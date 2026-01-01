from __future__ import annotations

from flask import jsonify


def register_system_routes(bp, store):
    @bp.route("/diagnostics", methods=["GET"])
    def diagnostics_overview():
        return jsonify({"success": True, "items": store.get_diagnostics()})

    @bp.route("/system/health", methods=["GET"])
    def system_health():
        return jsonify({"success": True, "data": store.get_system_health()})

    @bp.route("/system/resources", methods=["GET"])
    def system_resources():
        return jsonify({"success": True, "data": store.get_system_resources()})

    @bp.route("/system/uptime", methods=["GET"])
    def system_uptime():
        return jsonify({"success": True, "data": store.get_system_uptime()})


__all__ = ["register_system_routes"]
