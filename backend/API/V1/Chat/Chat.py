from flask import jsonify, request

_ACTION_RESPONSES = {
    "status": "Current mission: Inspect Zone A – 68% complete.",
    "dock": "Navigating back to docking station and preparing for charge.",
    "scan": "Beginning perimeter scan using lidar and stereo cameras.",
    "alerts": "No critical sensor alerts. Last warning cleared 12 min ago.",
    "start_recording": "Recording started. I will notify you when the bag is ready.",
    "stop_recording": "Recording stopped and upload queued to the server.",
}


def _build_robot_reply(text: str) -> str:
    lowered = text.lower()
    if "mission" in lowered:
        return "Mission status is nominal. Next checkpoint ETA 3 minutes."
    if "battery" in lowered:
        return "Battery at 78% with healthy temperature profile."
    if "stop" in lowered:
        return "Emergency stop is armed and can be triggered instantly."
    if "return" in lowered or "dock" in lowered:
        return _ACTION_RESPONSES["dock"]
    return "Acknowledged. Processing your request and will report back shortly."


def register_chat_routes(bp, store):
    @bp.route("/chat/history", methods=["GET"])
    def chat_history():
        return jsonify({"success": True, "items": store.list_chat_messages()})

    @bp.route("/chat/messages", methods=["POST"])
    def send_message():
        data = request.get_json(silent=True) or {}
        text = (data.get("text") or "").strip()
        if not text:
            return (
                jsonify({"success": False, "message": "Message text is required"}),
                400,
            )
        message = store.add_chat_message("human", text)
        robot_reply = store.add_chat_message("robot", _build_robot_reply(text))
        return jsonify({"success": True, "message": message, "reply": robot_reply})

    @bp.route("/chat/actions/<action>", methods=["POST"])
    def trigger_action(action):
        key = action.replace("-", "_").lower()
        reply_text = _ACTION_RESPONSES.get(
            key,
            f"Action '{action}' acknowledged. Executing standard workflow.",
        )
        robot_reply = store.add_chat_message("robot", reply_text)
        return jsonify({"success": True, "reply": robot_reply})


__all__ = ["register_chat_routes"]
