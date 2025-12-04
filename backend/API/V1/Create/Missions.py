from flask import jsonify, request


def register_mission_routes(bp, store):
    @bp.route("/missions", methods=["GET"])
    def list_missions():
        return jsonify({"success": True, "items": store.list_missions()})

    @bp.route("/missions", methods=["POST"])
    def create_mission():
        data = request.get_json(silent=True) or {}
        if not data.get("name"):
            return (
                jsonify({"success": False, "message": "Mission name is required"}),
                400,
            )
        item = store.create_mission(data)
        return jsonify({"success": True, "item": item}), 201

    @bp.route("/missions/<mission_id>", methods=["PUT", "PATCH"])
    def update_mission(mission_id):
        data = request.get_json(silent=True) or {}
        try:
            item = store.update_mission(mission_id, data)
            return jsonify({"success": True, "item": item})
        except KeyError:
            return (
                jsonify({"success": False, "message": "Mission not found"}),
                404,
            )
    
    @bp.route("/missions/<mission_id>", methods=["DELETE"])
    def delete_mission(mission_id):
        try:
            store.delete_mission(mission_id)
            return jsonify({"success": True})
        except KeyError:
            return (
                jsonify({"success": False, "message": "Mission not found"}),
                404,
            )

    @bp.route("/missions/<mission_id>/initiate", methods=["POST"])
    def initiate_mission(mission_id):
        try:
            item = store.initiate_mission(mission_id)
            return jsonify(
                {
                    "success": True,
                    "item": item,
                    "message": "Mission sent to robot",
                }
            )
        except KeyError:
            return (
                jsonify({"success": False, "message": "Mission not found"}),
                404,
            )


__all__ = ["register_mission_routes"]
