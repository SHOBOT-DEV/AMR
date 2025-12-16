from flask import jsonify


def register_stats_routes(bp, store):
    @bp.route("/stats", methods=["GET"])
    def stats_payload():
        return jsonify({"success": True, "data": store.get_stats()})


__all__ = ["register_stats_routes"]
