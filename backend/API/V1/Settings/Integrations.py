from flask import jsonify, request


def register_integration_routes(bp, store):
    @bp.route("/settings/integrations", methods=["GET"])
    def list_integrations():
        return jsonify({"success": True, "items": store.list_integrations()})

    @bp.route("/settings/integrations", methods=["POST"])
    def add_integration():
        data = request.get_json(silent=True) or {}
        if not data.get("name"):
            return (
                jsonify({"success": False, "message": "Name is required"}),
                400,
            )
        item = store.add_integration(data)
        return jsonify({"success": True, "item": item}), 201

    @bp.route("/settings/integrations/<integration_id>", methods=["PATCH"])
    def update_integration(integration_id):
        data = request.get_json(silent=True) or {}
        try:
            item = store.update_integration(integration_id, data)
            return jsonify({"success": True, "item": item})
        except KeyError:
            return (
                jsonify({"success": False, "message": "Integration not found"}),
                404,
            )


__all__ = ["register_integration_routes"]
