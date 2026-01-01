from __future__ import annotations

from flask import jsonify, request


def register_integration_routes(bp, store):
    @bp.route("/integrations", methods=["GET"])
    def list_integrations():
        return jsonify({"success": True, "items": store.list_integrations()})

    @bp.route("/integrations", methods=["POST"])
    def add_integration():
        data = request.get_json(silent=True) or {}
        if not data.get("name"):
            return jsonify({"success": False, "message": "Name is required"}), 400
        item = store.add_integration(data)
        return jsonify({"success": True, "item": item}), 201

    @bp.route("/integrations/<integration_id>", methods=["DELETE"])
    def delete_integration(integration_id: str):
        try:
            item = store.delete_integration(integration_id)
        except KeyError:
            return jsonify({"success": False, "message": "Integration not found"}), 404
        return jsonify({"success": True, "item": item})


__all__ = ["register_integration_routes"]
