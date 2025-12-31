from flask import jsonify, request


def register_maps_routes(bp, store):
    @bp.route("/maps", methods=["GET"])
    def list_maps():
        return jsonify({"success": True, "items": store.list_maps()})

    @bp.route("/maps", methods=["POST"])
    def create_map():
        data = request.get_json(silent=True) or {}
        if not data.get("name"):
            return (
                jsonify({"success": False, "message": "Map name is required"}),
                400,
            )
        item = store.create_map(data)
        return jsonify({"success": True, "item": item}), 201

    @bp.route("/maps/<map_id>", methods=["GET"])
    def fetch_map(map_id):
        try:
            item = store.get_map(map_id)
            return jsonify({"success": True, "item": item})
        except KeyError:
            return (
                jsonify({"success": False, "message": "Map not found"}),
                404,
            )

    @bp.route("/maps/<map_id>", methods=["PUT", "PATCH"])
    def update_map(map_id):
        data = request.get_json(silent=True) or {}
        try:
            item = store.update_map(map_id, data)
            return jsonify({"success": True, "item": item})
        except KeyError:
            return (
                jsonify({"success": False, "message": "Map not found"}),
                404,
            )

    @bp.route("/maps/<map_id>", methods=["DELETE"])
    def delete_map(map_id):
        try:
            store.delete_map(map_id)
            return jsonify({"success": True})
        except KeyError:
            return (
                jsonify({"success": False, "message": "Map not found"}),
                404,
            )

    @bp.route("/maps/<map_id>/load", methods=["POST"])
    def load_map(map_id):
        try:
            item = store.mark_map_loaded(map_id)
            store.set_current_map(map_id)
            return jsonify(
                {
                    "success": True,
                    "item": item,
                    "message": "Map dispatched to robot",
                }
            )
        except KeyError:
            return (
                jsonify({"success": False, "message": "Map not found"}),
                404,
            )

    @bp.route("/maps/load", methods=["POST"])
    def load_map_by_body():
        data = request.get_json(silent=True) or {}
        map_id = data.get("map_id") or data.get("id")
        if not map_id:
            return jsonify({"success": False, "message": "map_id is required"}), 400
        try:
            item = store.mark_map_loaded(map_id)
            store.set_current_map(map_id)
            return jsonify(
                {
                    "success": True,
                    "item": item,
                    "message": "Map dispatched to robot",
                }
            )
        except KeyError:
            return (
                jsonify({"success": False, "message": "Map not found"}),
                404,
            )

    @bp.route("/maps/save", methods=["POST"])
    def save_map():
        data = request.get_json(silent=True) or {}
        map_id = data.get("map_id") or data.get("id")
        if not map_id:
            return jsonify({"success": False, "message": "map_id is required"}), 400
        try:
            item = store.update_map(map_id, data)
            return jsonify({"success": True, "item": item})
        except KeyError:
            return (
                jsonify({"success": False, "message": "Map not found"}),
                404,
            )

    @bp.route("/maps/current", methods=["GET"])
    def current_map():
        item = store.get_current_map()
        if not item:
            return jsonify({"success": False, "message": "No current map"}), 404
        return jsonify({"success": True, "item": item})


__all__ = ["register_maps_routes"]
