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

    @bp.route("/maps/<map_id>/preview", methods=["GET"])
    def preview_map(map_id):
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

    @bp.route("/maps/load/<map_id>", methods=["POST"])
    def load_map_legacy(map_id):
        try:
            item = store.mark_map_loaded(map_id)
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

    @bp.route("/maps/data", methods=["GET"])
    def list_maps_with_data():
        return jsonify({"success": True, "items": store.get_maps_data()})

    @bp.route("/maps/edit/<map_id>", methods=["PUT", "PATCH"])
    def edit_map_overlay(map_id):
        data = request.get_json(silent=True) or {}
        save_as_new = request.args.get("saveAsNew", "false").lower() == "true"
        try:
            item = store.update_map_overlay(map_id, data, save_as_new)
            return jsonify({"success": True, "item": item})
        except KeyError:
            return (
                jsonify({"success": False, "message": "Map not found"}),
                404,
            )


__all__ = ["register_maps_routes"]
