from flask import jsonify, request


def register_diagnostics_routes(bp, store):
    @bp.route("/monitor/diagnostics", methods=["GET"])
    def monitor_diagnostics_overview():
        return jsonify({"success": True, "items": store.get_diagnostics()})

    @bp.route("/monitor/diagnostics/<diag_id>", methods=["PATCH"])
    def update_monitor_diagnostic(diag_id):
        data = request.get_json(silent=True) or {}
        try:
            item = store.update_diagnostic(diag_id, data)
            return jsonify({"success": True, "item": item})
        except KeyError:
            return (
                jsonify({"success": False, "message": "Panel not found"}),
                404,
            )


__all__ = ["register_diagnostics_routes"]
