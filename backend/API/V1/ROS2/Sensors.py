from __future__ import annotations

import base64

from flask import jsonify


def register_ros2_sensor_routes(bp, store):
    @bp.route("/ros2/sensors/scan", methods=["GET"])
    def ros2_scan():
        scan = store.get_ros2_scan()
        if scan is None:
            return jsonify({"success": False, "message": "No scan data available"}), 404
        return jsonify({"success": True, "scan": scan})

    @bp.route("/ros2/costmap", methods=["GET"])
    def ros2_costmap():
        costmap = store.get_ros2_costmap()
        if costmap is None:
            return jsonify({"success": False, "message": "Costmap not available"}), 503

        data = costmap.get("data", b"")
        if isinstance(data, list):
            data = bytes(data)
        if isinstance(data, str):
            encoded = data
        else:
            encoded = base64.b64encode(data).decode()

        return jsonify(
            {
                "frame_id": costmap.get("frame_id", "map"),
                "resolution": costmap.get("resolution", 0.0),
                "width": costmap.get("width", 0),
                "height": costmap.get("height", 0),
                "origin": costmap.get("origin", {"x": 0.0, "y": 0.0}),
                "timestamp": costmap.get("timestamp", ""),
                "data": encoded,
            }
        )


__all__ = ["register_ros2_sensor_routes"]
