from flask import jsonify, request


def register_waypoint_routes(bp, store):
    @bp.route("/waypoints", methods=["GET"])
    def list_waypoints():
        return jsonify({"success": True, "items": store.list_waypoints()})

    @bp.route("/waypoints", methods=["POST"])
    def create_waypoint():
        data = request.get_json(silent=True) or {}
        if not data.get("name"):
            return (
                jsonify({"success": False, "message": "Waypoint name is required"}),
                400,
            )
        item = store.create_waypoint(data)
        return jsonify({"success": True, "item": item}), 201

    @bp.route("/waypoints/<waypoint_id>", methods=["PUT", "PATCH"])
    def update_waypoint(waypoint_id):
        data = request.get_json(silent=True) or {}
        try:
            item = store.update_waypoint(waypoint_id, data)
            return jsonify({"success": True, "item": item})
        except KeyError:
            return (
                jsonify({"success": False, "message": "Waypoint not found"}),
                404,
            )

    @bp.route("/waypoints/<waypoint_id>", methods=["DELETE"])
    def delete_waypoint(waypoint_id):
        try:
            store.delete_waypoint(waypoint_id)
            return jsonify({"success": True})
        except KeyError:
            return (
                jsonify({"success": False, "message": "Waypoint not found"}),
                404,
            )


__all__ = ["register_waypoint_routes"]
