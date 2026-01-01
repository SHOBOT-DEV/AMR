from flask import jsonify


def register_stats_routes(bp, store):
    @bp.route("/stats", methods=["GET"])
    def stats_payload():
        return jsonify({"success": True, "data": store.get_stats()})

    @bp.route("/stats/usage", methods=["GET"])
    def stats_usage():
        return jsonify({"success": True, "data": store.get_stats_usage()})

    @bp.route("/stats/navigation", methods=["GET"])
    def stats_navigation():
        return jsonify({"success": True, "data": store.get_stats_navigation()})

    @bp.route("/stats/missions", methods=["GET"])
    def stats_missions():
        return jsonify({"success": True, "data": store.get_stats_missions()})


__all__ = ["register_stats_routes"]
