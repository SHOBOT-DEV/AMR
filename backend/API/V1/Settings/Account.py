from flask import jsonify, request


def register_account_routes(bp, store):
    @bp.route("/settings/account", methods=["GET"])
    def get_account_profile():
        return jsonify({"success": True, "data": store.get_account()})

    @bp.route("/settings/account", methods=["PATCH"])
    def update_account_profile():
        data = request.get_json(silent=True) or {}
        if not data:
            return (
                jsonify({"success": False, "message": "No fields provided"}),
                400,
            )
        updated = store.update_account(data)
        return jsonify({"success": True, "data": updated})


__all__ = ["register_account_routes"]
