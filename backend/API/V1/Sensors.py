from __future__ import annotations

from flask import jsonify


def register_sensor_routes(bp, store):
    @bp.route("/sensors", methods=["GET"])
    def list_sensors():
        return jsonify({"success": True, "items": store.list_sensors()})

    @bp.route("/sensors/status", methods=["GET"])
    def sensors_status():
        return jsonify({"success": True, "items": store.list_sensors()})

    @bp.route("/sensors/reset", methods=["POST"])
    def sensors_reset():
        items = store.reset_sensors()
        return jsonify({"success": True, "items": items})

    @bp.route("/sensors/<name>/status", methods=["GET"])
    def sensor_status(name: str):
        item = store.get_sensor_status(name)
        if not item:
            return jsonify({"success": False, "message": "Sensor not found"}), 404
        return jsonify({"success": True, "item": item})


__all__ = ["register_sensor_routes"]
