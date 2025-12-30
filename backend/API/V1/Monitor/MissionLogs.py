from flask import jsonify, request


def register_mission_log_routes(bp, store):
    @bp.route("/monitor/mission-logs", methods=["GET"])
    def list_mission_logs():
        return jsonify({"success": True, "items": store.get_mission_history()})

    @bp.route("/monitor/mission-logs", methods=["POST"])
    def add_mission_log():
        data = request.get_json(silent=True) or {}
        if not data.get("mission"):
            return (
                jsonify({"success": False, "message": "Mission name is required"}),
                400,
            )
        item = store.add_mission_history(data)
        return jsonify({"success": True, "item": item}), 201

    @bp.route("/missions/logs", methods=["GET"])
    def list_mission_logs_legacy():
        return jsonify({"success": True, "items": store.get_mission_history()})

    @bp.route("/missions/logs/<log_id>/progress", methods=["GET"])
    def mission_log_progress(log_id):
        progress = store.get_mission_log_progress(log_id)
        if not progress:
            return (
                jsonify({"success": False, "message": "Mission log not found"}),
                404,
            )
        return jsonify({"success": True, "item": progress})


__all__ = ["register_mission_log_routes"]
