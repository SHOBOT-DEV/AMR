from __future__ import annotations

from flask import jsonify


def register_costmap_routes(bp, store):
    @bp.route("/costmap", methods=["GET"])
    def costmap_root():
        costmap = store.get_ros2_costmap()
        if costmap is None:
            return jsonify({"success": False, "message": "Costmap not available"}), 503
        return jsonify({"success": True, "costmap": costmap})

    @bp.route("/costmap/local", methods=["GET"])
    def costmap_local():
        costmap = store.get_ros2_costmap()
        if costmap is None:
            return jsonify({"success": False, "message": "Costmap not available"}), 503
        return jsonify({"success": True, "costmap": costmap})

    @bp.route("/costmap/global", methods=["GET"])
    def costmap_global():
        costmap = store.get_ros2_costmap()
        if costmap is None:
            return jsonify({"success": False, "message": "Costmap not available"}), 503
        return jsonify({"success": True, "costmap": costmap})

    @bp.route("/costmap/reset", methods=["POST"])
    def costmap_reset():
        store.set_ros2_costmap({"reset": True})
        return jsonify({"success": True, "message": "Costmap reset queued"})


__all__ = ["register_costmap_routes"]
