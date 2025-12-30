from flask import jsonify, request


def register_legacy_routes(bp, store):
    @bp.route("/", methods=["GET"])
    def api_home():
        return jsonify({"success": True, "message": "ANSCER ANYA API v1"})

    @bp.route("/auth/", methods=["POST"])
    def auth_login():
        data = request.get_json(silent=True) or {}
        if not data.get("email") or not data.get("password"):
            return (
                jsonify({"success": False, "message": "Email and password are required"}),
                400,
            )
        return jsonify(
            {
                "success": True,
                "token": "mock-jwt-token",
                "user": {"email": data.get("email")},
            }
        )

    # Robot bags
    @bp.route("/robot/bags/files/", methods=["GET"])
    def list_robot_bag_files():
        return jsonify({"success": True, "data": store.get_robot_bag_files()})

    @bp.route("/robot/bags/file/", methods=["GET"])
    def list_robot_bag_entries():
        return jsonify({"success": True, "items": store.get_robot_bags()})

    @bp.route("/robot/bags/downloads/<path:bag_path>", methods=["GET"])
    def download_robot_bag(bag_path):
        return jsonify(
            {
                "success": True,
                "message": "Download ready",
                "path": bag_path,
            }
        )

    # Quick goal
    @bp.route("/robot/quickgoal/", methods=["POST"])
    def quick_goal():
        data = request.get_json(silent=True) or {}
        return jsonify({"success": True, "message": "Quick goal sent", "data": data})

    @bp.route("/robot/quickgoal/<waypoint_id>", methods=["POST"])
    def quick_goal_waypoint(waypoint_id):
        return jsonify(
            {
                "success": True,
                "message": "Quick goal sent",
                "waypointId": waypoint_id,
            }
        )

    # Graphs
    @bp.route("/graphs", methods=["GET"])
    def list_graphs():
        return jsonify({"success": True, "items": store.list_graphs()})

    @bp.route("/graphs", methods=["POST"])
    def create_graph():
        data = request.get_json(silent=True) or {}
        if not data.get("name"):
            return (
                jsonify({"success": False, "message": "Graph name is required"}),
                400,
            )
        item = store.create_graph(data)
        return jsonify({"success": True, "item": item}), 201

    @bp.route("/graphs/<graph_id>", methods=["GET"])
    def get_graph(graph_id):
        try:
            item = store.get_graph(graph_id)
            return jsonify({"success": True, "item": item})
        except KeyError:
            return (
                jsonify({"success": False, "message": "Graph not found"}),
                404,
            )

    @bp.route("/graphs/<graph_id>", methods=["PUT", "PATCH"])
    def update_graph(graph_id):
        data = request.get_json(silent=True) or {}
        try:
            item = store.update_graph(graph_id, data)
            return jsonify({"success": True, "item": item})
        except KeyError:
            return (
                jsonify({"success": False, "message": "Graph not found"}),
                404,
            )

    @bp.route("/graphs/<graph_id>", methods=["DELETE"])
    def delete_graph(graph_id):
        try:
            store.delete_graph(graph_id)
            return jsonify({"success": True})
        except KeyError:
            return (
                jsonify({"success": False, "message": "Graph not found"}),
                404,
            )

    # Logs
    @bp.route("/logs", methods=["GET"])
    def list_logs():
        return jsonify({"success": True, "items": store.get_logs()})

    @bp.route("/logs/export", methods=["GET"])
    def export_logs():
        items = store.get_logs()
        return jsonify({"success": True, "items": items, "export": "csv"})

    @bp.route("/logs", methods=["POST"])
    def create_log():
        data = request.get_json(silent=True) or {}
        if not data.get("message"):
            return (
                jsonify({"success": False, "message": "Log message is required"}),
                400,
            )
        item = store.add_log(data)
        return jsonify({"success": True, "item": item}), 201

    # Analytics
    @bp.route("/analytics/battery", methods=["GET"])
    def battery_analytics():
        return jsonify({"success": True, "items": store.get_battery_analytics()})

    # Registers
    @bp.route("/registers/", methods=["GET"])
    def list_registers():
        return jsonify({"success": True, "items": store.get_registers()})

    # Downloads
    @bp.route("/downloads", methods=["GET"])
    def list_downloads():
        return jsonify({"success": True, "items": store.list_downloads()})

    @bp.route("/downloads/<path:filename>", methods=["GET"])
    def download_file(filename):
        entry = store.get_download(filename)
        if not entry:
            return (
                jsonify({"success": False, "message": "File not found"}),
                404,
            )
        return jsonify({"success": True, "item": entry})


__all__ = ["register_legacy_routes"]
