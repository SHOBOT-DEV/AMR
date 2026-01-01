from __future__ import annotations

from flask import jsonify


def register_perception_routes(bp, store):
    @bp.route("/perception/objects", methods=["GET"])
    def perception_objects():
        return jsonify({"success": True, "items": store.list_perception_objects()})

    @bp.route("/perception/humans", methods=["GET"])
    def perception_humans():
        return jsonify({"success": True, "items": store.list_perception_humans()})

    @bp.route("/perception/obstacles", methods=["GET"])
    def perception_obstacles():
        return jsonify({"success": True, "items": store.list_perception_obstacles()})

    @bp.route("/perception/status", methods=["GET"])
    def perception_status():
        return jsonify({"success": True, "data": store.get_perception_status()})


__all__ = ["register_perception_routes"]
