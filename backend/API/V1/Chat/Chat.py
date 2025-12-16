import json
import os
from typing import Any, Dict, Optional

import requests
from flask import jsonify, request

LLM_API_URL = os.getenv("LLM_API_URL")  # e.g. https://api.openai.com/v1/chat/completions
LLM_API_KEY = os.getenv("LLM_API_KEY")
LLM_MODEL = os.getenv("LLM_MODEL", "gpt-4o-mini")

_ACTION_RESPONSES = {
    "status": "Current mission: Inspect Zone A – 68% complete.",
    "dock": "Navigating back to docking station and preparing for charge.",
    "scan": "Beginning perimeter scan using lidar and stereo cameras.",
    "alerts": "No critical sensor alerts. Last warning cleared 12 min ago.",
    "start_recording": "Recording started. I will notify you when the bag is ready.",
    "stop_recording": "Recording stopped and upload queued to the server.",
}

_INTENT_ACTIONS = {
    "mission_status": {"action": "mission_status", "description": "Query active mission progress"},
    "return_to_dock": {"action": "dock", "description": "Send robot to dock"},
    "perimeter_scan": {"action": "scan", "description": "Start perimeter scan"},
    "sensor_alerts": {"action": "alerts", "description": "Fetch latest sensor alerts"},
    "start_recording": {"action": "start_recording", "description": "Begin rosbag recording"},
    "stop_recording": {"action": "stop_recording", "description": "Stop rosbag recording"},
}


def _call_llm_agent(user_text: str) -> Optional[Dict[str, Any]]:
    """Optional LLM call for intent + response. Returns parsed dict or None on failure."""
    if not (LLM_API_URL and LLM_API_KEY):
        return None

    system_prompt = (
        "You are a ROS2 AMR operations assistant. "
        "Return concise responses plus an intent name from this list: "
        "[mission_status, return_to_dock, perimeter_scan, sensor_alerts, start_recording, stop_recording, telemetry, smalltalk]. "
        "If you need to trigger a robot action, set action to one of [dock, scan, alerts, start_recording, stop_recording]. "
        "Respond in JSON with keys: intent, response, action (optional). Keep response under 2 sentences."
    )
    payload = {
        "model": LLM_MODEL,
        "messages": [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_text},
        ],
        "response_format": {"type": "json_object"},
        "temperature": 0.2,
    }
    headers = {
        "Authorization": f"Bearer {LLM_API_KEY}",
        "Content-Type": "application/json",
    }
    try:
        resp = requests.post(LLM_API_URL, json=payload, headers=headers, timeout=8)
        if resp.status_code >= 400:
            return None
        data = resp.json()
        content = data.get("choices", [{}])[0].get("message", {}).get("content")
        if not content:
            return None
        return json.loads(content)
    except Exception:
        return None


def _heuristic_intent(text: str) -> Dict[str, Any]:
    lowered = text.lower()
    if any(w in lowered for w in ["mission", "status", "progress"]):
        return {"intent": "mission_status", "response": _ACTION_RESPONSES["status"], "action": None}
    if any(w in lowered for w in ["dock", "charge", "return"]):
        return {"intent": "return_to_dock", "response": _ACTION_RESPONSES["dock"], "action": "dock"}
    if any(w in lowered for w in ["scan", "patrol", "perimeter"]):
        return {"intent": "perimeter_scan", "response": _ACTION_RESPONSES["scan"], "action": "scan"}
    if any(w in lowered for w in ["alert", "warning", "fault"]):
        return {"intent": "sensor_alerts", "response": _ACTION_RESPONSES["alerts"], "action": "alerts"}
    if "start recording" in lowered or "record" in lowered:
        return {"intent": "start_recording", "response": _ACTION_RESPONSES["start_recording"], "action": "start_recording"}
    if "stop recording" in lowered or "stop log" in lowered:
        return {"intent": "stop_recording", "response": _ACTION_RESPONSES["stop_recording"], "action": "stop_recording"}
    if "battery" in lowered:
        return {"intent": "telemetry", "response": "Battery at 78% with healthy temperature profile.", "action": None}
    return {"intent": "smalltalk", "response": "Acknowledged. Processing your request and will report back shortly.", "action": None}


def _build_robot_reply(text: str) -> Dict[str, Any]:
    agent = _call_llm_agent(text)
    if agent and isinstance(agent, dict):
        response_text = agent.get("response") or "Working on that now."
        intent = agent.get("intent") or "smalltalk"
        action = agent.get("action")
    else:
        heur = _heuristic_intent(text)
        response_text = heur["response"]
        intent = heur["intent"]
        action = heur["action"]

    actions = []
    if action:
        mapped = _INTENT_ACTIONS.get(intent) or {"action": action, "description": "Execute requested task"}
        actions.append(mapped)

    return {
        "text": response_text,
        "intent": intent,
        "actions": actions,
        "source": "llm" if agent else "rule",
    }


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

        reply_payload = _build_robot_reply(text)
        reply_text = reply_payload.get("text", "Acknowledged.")
        metadata = {
            "intent": reply_payload.get("intent"),
            "source": reply_payload.get("source"),
            "actions": reply_payload.get("actions", []),
        }
        robot_reply = store.add_chat_message("robot", reply_text, metadata=metadata)
        response = {
            "success": True,
            "message": message,
            "reply": robot_reply,
            "intent": metadata["intent"],
            "actions": metadata["actions"],
            "source": metadata["source"],
        }
        return jsonify(response)

    @bp.route("/chat/actions/<action>", methods=["POST"])
    def trigger_action(action):
        key = action.replace("-", "_").lower()
        reply_text = _ACTION_RESPONSES.get(
            key,
            f"Action '{action}' acknowledged. Executing standard workflow.",
        )
        metadata = {"intent": key, "source": "direct", "actions": [{"action": key, "description": "Direct action"}]}
        robot_reply = store.add_chat_message("robot", reply_text, metadata=metadata)
        return jsonify({"success": True, "reply": robot_reply, "intent": key, "actions": metadata["actions"]})


__all__ = ["register_chat_routes"]
