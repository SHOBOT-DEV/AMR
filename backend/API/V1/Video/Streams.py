from __future__ import annotations

from flask import jsonify


def register_video_routes(bp, store):
    @bp.route("/video/streams", methods=["GET"])
    def list_video_streams():
        return jsonify({"streams": store.list_video_streams()})

    @bp.route("/video/stream/<camera_id>", methods=["GET"])
    def get_video_stream(camera_id: str):
        stream = store.get_video_stream(camera_id)
        if not stream:
            return jsonify({"success": False, "message": "Camera not found"}), 404
        return jsonify({"stream": stream})


__all__ = ["register_video_routes"]
