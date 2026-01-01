from __future__ import annotations

from flask import jsonify, request


def register_recovery_routes(bp, store):
    @bp.route("/recovery/options", methods=["GET"])
    def recovery_options():
        return jsonify({"success": True, "items": store.list_recovery_options()})

    @bp.route("/recovery/execute", methods=["POST"])
    def recovery_execute():
        data = request.get_json(silent=True) or {}
        option_id = data.get("id") or data.get("option_id")
        if not option_id:
            return jsonify({"success": False, "message": "Recovery option id is required"}), 400
        action = store.execute_recovery(option_id)
        return jsonify({"success": True, "action": action})

    @bp.route("/recovery/status", methods=["GET"])
    def recovery_status():
        return jsonify({"success": True, "data": store.get_recovery_status()})


__all__ = ["register_recovery_routes"]
