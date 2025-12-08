from flask import jsonify, request


def register_log_routes(bp, store):
    @bp.route("/monitor/logs", methods=["GET"])
    def list_logs():
        return jsonify({"success": True, "items": store.get_logs()})

    @bp.route("/monitor/logs", methods=["POST"])
    def add_log():
        data = request.get_json(silent=True) or {}
        if not data.get("message"):
            return (
                jsonify({"success": False, "message": "Log message is required"}),
                400,
            )
        item = store.add_log(data)
        return jsonify({"success": True, "item": item}), 201


__all__ = ["register_log_routes"]
