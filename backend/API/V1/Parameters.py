from __future__ import annotations

from flask import jsonify, request


def register_parameter_routes(bp, store):
    @bp.route("/parameters", methods=["GET"])
    def list_parameters():
        return jsonify({"success": True, "parameters": store.list_ros2_parameters()})

    @bp.route("/parameters/<name>", methods=["GET"])
    def get_parameter(name: str):
        params = store.list_ros2_parameters()
        if name not in params:
            return jsonify({"success": False, "message": "Parameter not found"}), 404
        return jsonify({"success": True, "parameter": {name: params[name]}})

    @bp.route("/parameters", methods=["POST"])
    def set_parameters():
        payload = request.get_json(silent=True) or {}
        if not isinstance(payload, dict):
            return jsonify({"success": False, "message": "Parameters payload must be an object"}), 400
        updated = store.set_ros2_parameters(payload)
        return jsonify({"success": True, "parameters": updated})


__all__ = ["register_parameter_routes"]
