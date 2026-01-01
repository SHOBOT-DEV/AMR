from __future__ import annotations

from flask import jsonify, request


def register_mission_routes(bp, store):
    @bp.route("/missions", methods=["GET"])
    def list_missions():
        return jsonify({"success": True, "items": store.list_missions()})

    @bp.route("/missions", methods=["POST"])
    def create_mission():
        data = request.get_json(silent=True) or {}
        item = store.create_mission(data)
        return jsonify({"success": True, "item": item}), 201

    @bp.route("/missions/<mission_id>", methods=["GET"])
    def get_mission(mission_id: str):
        try:
            item = store.get_mission(mission_id)
        except KeyError:
            return jsonify({"success": False, "message": "Mission not found"}), 404
        return jsonify({"success": True, "item": item})

    @bp.route("/missions/<mission_id>", methods=["PUT"])
    def update_mission(mission_id: str):
        data = request.get_json(silent=True) or {}
        try:
            item = store.update_mission(mission_id, data)
        except KeyError:
            return jsonify({"success": False, "message": "Mission not found"}), 404
        return jsonify({"success": True, "item": item})

    @bp.route("/missions/<mission_id>", methods=["DELETE"])
    def delete_mission(mission_id: str):
        try:
            store.delete_mission(mission_id)
        except KeyError:
            return jsonify({"success": False, "message": "Mission not found"}), 404
        return jsonify({"success": True})

    @bp.route("/missions/<mission_id>/start", methods=["POST"])
    def start_mission(mission_id: str):
        try:
            item = store.initiate_mission(mission_id)
        except KeyError:
            return jsonify({"success": False, "message": "Mission not found"}), 404
        return jsonify({"success": True, "item": item})

    @bp.route("/missions/<mission_id>/pause", methods=["POST"])
    def pause_mission(mission_id: str):
        try:
            item = store.pause_mission(mission_id)
        except KeyError:
            return jsonify({"success": False, "message": "Mission not found"}), 404
        return jsonify({"success": True, "item": item})

    @bp.route("/missions/<mission_id>/resume", methods=["POST"])
    def resume_mission(mission_id: str):
        try:
            item = store.resume_mission(mission_id)
        except KeyError:
            return jsonify({"success": False, "message": "Mission not found"}), 404
        return jsonify({"success": True, "item": item})

    @bp.route("/missions/<mission_id>/cancel", methods=["POST"])
    def cancel_mission(mission_id: str):
        try:
            item = store.cancel_mission(mission_id)
        except KeyError:
            return jsonify({"success": False, "message": "Mission not found"}), 404
        return jsonify({"success": True, "item": item})

    @bp.route("/missions/status", methods=["GET"])
    def mission_status():
        return jsonify({"success": True, "items": store.get_mission_statuses()})

    @bp.route("/missions/history", methods=["GET"])
    def mission_history():
        return jsonify({"success": True, "items": store.get_mission_history()})


__all__ = ["register_mission_routes"]
