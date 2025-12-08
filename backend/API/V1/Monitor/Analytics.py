from flask import jsonify, request


def register_analytics_routes(bp, store):
    @bp.route("/monitor/analytics", methods=["GET"])
    def get_analytics():
        return jsonify({"success": True, "data": store.get_analytics()})

    @bp.route("/monitor/analytics/alerts", methods=["POST"])
    def add_alert():
        data = request.get_json(silent=True) or {}
        if not data.get("title"):
            return (
                jsonify({"success": False, "message": "Alert title is required"}),
                400,
            )
        alert = store.add_analytics_alert(data)
        return jsonify({"success": True, "item": alert}), 201

    @bp.route("/monitor/analytics/summary/<label>", methods=["PATCH"])
    def update_summary(label):
        data = request.get_json(silent=True) or {}
        try:
            item = store.update_analytics_summary(label, data)
            return jsonify({"success": True, "item": item})
        except KeyError:
            return (
                jsonify({"success": False, "message": "Metric not found"}),
                404,
            )

    @bp.route("/monitor/analytics/series", methods=["PUT"])
    def override_series():
        data = request.get_json(silent=True) or {}
        series = data.get("series")
        if not isinstance(series, list):
            return (
                jsonify({"success": False, "message": "Series must be a list"}),
                400,
            )
        updated = store.set_analytics_series(series)
        return jsonify({"success": True, "series": updated})


__all__ = ["register_analytics_routes"]
