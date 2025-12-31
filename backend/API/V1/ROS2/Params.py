from __future__ import annotations

from flask import jsonify, request


def register_ros2_param_routes(bp, store):
    @bp.route("/ros2/parameters", methods=["GET"])
    def ros2_list_parameters():
        return jsonify(store.list_ros2_parameters())

    @bp.route("/ros2/parameters", methods=["POST"])
    def ros2_set_parameters():
        payload = request.get_json(silent=True) or {}
        if not isinstance(payload, dict):
            return jsonify({"success": False, "message": "Parameters payload must be an object"}), 400
        updated = store.set_ros2_parameters(payload)
        return jsonify({"success": True, "parameters": updated})


__all__ = ["register_ros2_param_routes"]
