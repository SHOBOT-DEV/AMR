from flask import jsonify, request


def register_robot_bag_routes(bp, store):
    @bp.route("/monitor/robot-bags", methods=["GET"])
    def list_robot_bags():
        return jsonify({"success": True, "items": store.get_robot_bags()})

    @bp.route("/monitor/robot-bags", methods=["POST"])
    def add_robot_bag():
        data = request.get_json(silent=True) or {}
        if not data.get("name"):
            return (
                jsonify({"success": False, "message": "Bag filename is required"}),
                400,
            )
        item = store.add_robot_bag(data)
        return jsonify({"success": True, "item": item}), 201


__all__ = ["register_robot_bag_routes"]
