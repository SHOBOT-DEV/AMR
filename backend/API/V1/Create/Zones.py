from flask import jsonify, request


def register_zone_routes(bp, store):
    @bp.route("/zones", methods=["GET"])
    def list_zones():
        return jsonify({"success": True, "items": store.list_zones()})

    @bp.route("/zones", methods=["POST"])
    def create_zone():
        data = request.get_json(silent=True) or {}
        if not data.get("name"):
            return (
                jsonify({"success": False, "message": "Zone name is required"}),
                400,
            )
        item = store.create_zone(data)
        return jsonify({"success": True, "item": item}), 201

    @bp.route("/zones/<zone_id>", methods=["PUT", "PATCH"])
    def update_zone(zone_id):
        data = request.get_json(silent=True) or {}
        try:
            item = store.update_zone(zone_id, data)
            return jsonify({"success": True, "item": item})
        except KeyError:
            return (
                jsonify({"success": False, "message": "Zone not found"}),
                404,
            )

    @bp.route("/zones/<zone_id>", methods=["DELETE"])
    def delete_zone(zone_id):
        try:
            store.delete_zone(zone_id)
            return jsonify({"success": True})
        except KeyError:
            return (
                jsonify({"success": False, "message": "Zone not found"}),
                404,
            )


__all__ = ["register_zone_routes"]
