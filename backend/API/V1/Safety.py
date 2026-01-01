from __future__ import annotations

from flask import jsonify


def register_safety_routes(bp, store):
    @bp.route("/safety/status", methods=["GET"])
    def safety_status():
        return jsonify(
            {
                "success": True,
                "stop": store.get_ros2_safety_stop(),
                "acknowledged": store.get_safety_acknowledged(),
            }
        )

    @bp.route("/safety/stop", methods=["POST"])
    def safety_stop():
        store.set_ros2_safety_stop(True)
        return jsonify({"success": True, "stop": True})

    @bp.route("/safety/reset", methods=["POST"])
    def safety_reset():
        status = store.reset_safety()
        return jsonify({"success": True, **status})

    @bp.route("/safety/acknowledge", methods=["POST"])
    def safety_acknowledge():
        store.acknowledge_safety()
        return jsonify({"success": True, "acknowledged": True})


__all__ = ["register_safety_routes"]
