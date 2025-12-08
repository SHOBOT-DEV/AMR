from flask import jsonify, request


def register_appearance_routes(bp, store):
    @bp.route("/settings/appearance", methods=["GET"])
    def get_theme():
        return jsonify({"success": True, "data": store.get_appearance()})

    @bp.route("/settings/appearance", methods=["PATCH"])
    def update_theme():
        data = request.get_json(silent=True) or {}
        updated = store.update_appearance(data)
        return jsonify({"success": True, "data": updated})


__all__ = ["register_appearance_routes"]
