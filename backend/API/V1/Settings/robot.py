from flask import jsonify, request


def register_robot_settings_routes(bp, store):
    @bp.route("/settings/robot", methods=["GET"])
    def get_robot_settings():
        return jsonify({"success": True, "data": store.get_robot_settings()})

    @bp.route("/settings/robot", methods=["PATCH"])
    def update_robot_settings():
        data = request.get_json(silent=True) or {}
        updated = store.update_robot_settings(data)
        return jsonify({"success": True, "data": updated})


__all__ = ["register_robot_settings_routes"]
