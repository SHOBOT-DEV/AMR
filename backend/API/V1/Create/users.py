from flask import jsonify, request


def register_user_routes(bp, store):
    @bp.route("/users", methods=["GET"])
    def list_users():
        return jsonify({"success": True, "items": store.list_users()})

    @bp.route("/users", methods=["POST"])
    def create_user():
        data = request.get_json(silent=True) or {}
        if not data.get("name") or not data.get("email"):
            return (
                jsonify({"success": False, "message": "Name and email are required"}),
                400,
            )
        item = store.create_user(data)
        return jsonify({"success": True, "item": item}), 201

    @bp.route("/users/<user_id>", methods=["GET"])
    def get_user(user_id):
        try:
            item = store.get_user(user_id)
            return jsonify({"success": True, "item": item})
        except KeyError:
            return (
                jsonify({"success": False, "message": "User not found"}),
                404,
            )

    @bp.route("/users/<user_id>", methods=["PUT", "PATCH"])
    def update_user(user_id):
        data = request.get_json(silent=True) or {}
        try:
            item = store.update_user(user_id, data)
            return jsonify({"success": True, "item": item})
        except KeyError:
            return (
                jsonify({"success": False, "message": "User not found"}),
                404,
            )

    @bp.route("/users/<user_id>", methods=["DELETE"])
    def delete_user(user_id):
        try:
            store.delete_user(user_id)
            return jsonify({"success": True})
        except KeyError:
            return (
                jsonify({"success": False, "message": "User not found"}),
                404,
            )

    @bp.route("/users/<user_id>/reset-password", methods=["POST"])
    def reset_password(user_id):
        try:
            item = store.reset_user_password(user_id)
            return jsonify(
                {
                    "success": True,
                    "item": item,
                    "message": "Password reset token generated",
                }
            )
        except KeyError:
            return (
                jsonify({"success": False, "message": "User not found"}),
                404,
            )

    @bp.route("/users/current", methods=["GET"])
    def current_user():
        profile = store.get_account()
        return jsonify({"success": True, "item": profile})


__all__ = ["register_user_routes"]
