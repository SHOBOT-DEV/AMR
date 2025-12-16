from flask import jsonify, request


def register_security_routes(bp, store):
    @bp.route("/settings/security", methods=["GET"])
    def get_security_flags():
        return jsonify({"success": True, "data": store.get_security()})

    @bp.route("/settings/security", methods=["PATCH"])
    def update_security_flags():
        data = request.get_json(silent=True) or {}
        updated = store.update_security(data)
        return jsonify({"success": True, "data": updated})

    @bp.route("/settings/security/events", methods=["GET"])
    def list_security_events():
        return jsonify({"success": True, "items": store.list_security_events()})

    @bp.route("/settings/security/events", methods=["POST"])
    def create_security_event():
        data = request.get_json(silent=True) or {}
        if not data.get("action"):
            return (
                jsonify({"success": False, "message": "Action is required"}),
                400,
            )
        item = store.add_security_event(data)
        return jsonify({"success": True, "item": item}), 201


__all__ = ["register_security_routes"]
