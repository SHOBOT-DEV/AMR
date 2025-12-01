"""In-memory data used by the v1 mock API.

The current frontend consumes synthetic data while the real robot/telemetry
pipeline is being built.  Instead of hard-coding those values inside each
route, we keep them in a single data store that exposes small helper methods
to read/update the collections.  This keeps the view functions tidy and makes
it trivial to swap the implementation with a database later on.
"""

from __future__ import annotations

from copy import deepcopy
from datetime import datetime
from threading import Lock
from typing import Any, Dict, List, Tuple
from uuid import uuid4


def _utc_ts() -> str:
    return datetime.utcnow().isoformat(timespec="seconds") + "Z"


def _clone(value: Any) -> Any:
    return deepcopy(value)


class FrontendDataStore:
    """Minimal state container for the mock API."""

    def __init__(self) -> None:
        self._lock = Lock()
        self._state: Dict[str, Any] = self._default_state()

    # ------------------------------------------------------------------
    # helpers
    def _new_id(self, prefix: str) -> str:
        return f"{prefix}_{uuid4().hex[:8]}"

    def _default_state(self) -> Dict[str, Any]:
        return {
            "maps": [
                {
                    "id": "cfl_gf",
                    "name": "CFL_GF",
                    "createdBy": "CNDE IITM",
                    "status": "Active",
                    "createdAt": "2025-11-12T08:30:00Z",
                    "previewImage": "",
                },
                {
                    "id": "shobot_arena",
                    "name": "shobot_arena",
                    "createdBy": "ANSCER ADMIN",
                    "status": "",
                    "createdAt": "2025-11-10T05:12:00Z",
                    "previewImage": "/images/maps/shobot_arena.png",
                },
                {
                    "id": "shobot_arena2",
                    "name": "shobot_arena2",
                    "createdBy": "ANSCER ADMIN",
                    "status": "",
                    "createdAt": "2025-11-09T18:44:00Z",
                    "previewImage": "/images/maps/shobot_arena2.png",
                },
                {
                    "id": "zones",
                    "name": "Zones",
                    "createdBy": "ANSCER ADMIN",
                    "status": "",
                    "createdAt": "2025-11-09T18:40:00Z",
                    "previewImage": "",
                },
                {
                    "id": "waypoints",
                    "name": "Waypoints",
                    "createdBy": "ANSCER ADMIN",
                    "status": "",
                    "createdAt": "2025-11-09T18:41:00Z",
                    "previewImage": "",
                },
                {
                    "id": "users",
                    "name": "Users",
                    "createdBy": "ANSCER ADMIN",
                    "status": "",
                    "createdAt": "2025-11-09T18:42:00Z",
                    "previewImage": "",
                },
            ],
            "zones": [
                {
                    "id": "z1",
                    "name": "Assembly Lane",
                    "category": "Safe",
                    "active": True,
                    "geometry": "Polygon(32,18…)",
                    "createdAt": "2025-11-17",
                },
                {
                    "id": "z2",
                    "name": "Battery Bay",
                    "category": "No-Go",
                    "active": True,
                    "geometry": "Polygon(27,11…)",
                    "createdAt": "2025-11-15",
                },
                {
                    "id": "z3",
                    "name": "Dock Tunnel",
                    "category": "Caution",
                    "active": False,
                    "geometry": "Line(82,90…)",
                    "createdAt": "2025-11-14",
                },
            ],
            "waypoints": [
                {
                    "id": "wp1",
                    "name": "WP_A",
                    "category": "Nav",
                    "active": True,
                    "geom": "Point(12 34)",
                    "createdAt": "2025-11-17",
                    "notes": "First waypoint",
                },
                {
                    "id": "wp2",
                    "name": "WP_B",
                    "category": "Inspect",
                    "active": False,
                    "geom": "Point(98 76)",
                    "createdAt": "2025-11-17",
                    "notes": "Inspection point",
                },
                {
                    "id": "wp3",
                    "name": "WP_C",
                    "category": "Charge",
                    "active": True,
                    "geom": "Point(44 55)",
                    "createdAt": "2025-11-16",
                    "notes": "Charging pad",
                },
            ],
            "missions": [
                {
                    "id": "m1",
                    "name": "Inspect Zone A",
                    "owner": "CNDE",
                    "status": "Draft",
                    "createdAt": "2025-11-17",
                    "notes": "Routine inspection",
                },
                {
                    "id": "m2",
                    "name": "Delivery Route 3",
                    "owner": "ANSCER ADMIN",
                    "status": "Scheduled",
                    "createdAt": "2025-11-16",
                    "notes": "Delivery to docks",
                },
                {
                    "id": "m3",
                    "name": "Battery Check",
                    "owner": "CNDE",
                    "status": "Completed",
                    "createdAt": "2025-11-15",
                    "notes": "Post-run check",
                },
            ],
            "users": [
                {
                    "id": "u1",
                    "name": "ANSCER ADMIN",
                    "email": "admin@anscer.com",
                    "role": "Admin",
                    "status": "Active",
                    "createdBy": "—",
                    "createdAt": "—",
                },
                {
                    "id": "u2",
                    "name": "CNDE",
                    "email": "cnde@iitm.org",
                    "role": "Admin",
                    "status": "Active",
                    "createdBy": "ANSCER ADMIN",
                    "createdAt": "02/18/2025, 5:37:51 PM",
                },
            ],
            "analytics": {
                "summary": [
                    {"label": "Incidents", "value": 2, "trend": "+1 vs last week"},
                    {"label": "Stops Issued", "value": 14, "trend": "-3 vs last week"},
                    {"label": "Battery Swaps", "value": 5, "trend": "Stable"},
                    {"label": "Avg. Cycle", "value": "42 min", "trend": "±0"},
                ],
                "series": [12, 18, 22, 16, 24, 26, 20],
                "alerts": [
                    {
                        "id": "alert1",
                        "title": "Obstacle spikes",
                        "detail": "Lidar reported 5 high-density events on Dock Tunnel.",
                    },
                    {
                        "id": "alert2",
                        "title": "Slow return",
                        "detail": "Mission Delivery Route 3 exceeded SLA by 4 min.",
                    },
                ],
            },
            "diagnostics": [
                {
                    "id": "battery",
                    "title": "Battery Health",
                    "value": "93%",
                    "status": "Nominal",
                    "detail": "Cells balanced",
                },
                {
                    "id": "motors",
                    "title": "Drive Motors",
                    "value": "Temp 48°C",
                    "status": "Monitoring",
                    "detail": "Torque variance +3%",
                },
                {
                    "id": "sensors",
                    "title": "Sensor Suite",
                    "value": "All online",
                    "status": "Nominal",
                    "detail": "Last calibration 12h ago",
                },
            ],
            "logs": [
                {
                    "id": "log1",
                    "ts": "10:42:01",
                    "system": "Navigation",
                    "message": "Replanned path around blocked aisle",
                    "level": "info",
                },
                {
                    "id": "log2",
                    "ts": "10:15:22",
                    "system": "Safety",
                    "message": "Emergency stop acknowledged",
                    "level": "warn",
                },
                {
                    "id": "log3",
                    "ts": "09:57:10",
                    "system": "Battery",
                    "message": "Pack voltage dipped to 45.9V",
                    "level": "warn",
                },
            ],
            "mission_logs": [
                {
                    "id": "mh1",
                    "mission": "Inspect Zone A",
                    "window": "08:00–08:18",
                    "outcome": "Completed",
                    "notes": "No issues",
                },
                {
                    "id": "mh2",
                    "mission": "Delivery Route 3",
                    "window": "08:30–09:10",
                    "outcome": "Delayed",
                    "notes": "Obstacle at Dock Tunnel",
                },
                {
                    "id": "mh3",
                    "mission": "Battery Check",
                    "window": "09:15–09:32",
                    "outcome": "Completed",
                    "notes": "Pack swap verified",
                },
            ],
            "robot_bags": [
                {
                    "id": "bag1",
                    "name": "mission-0915.bag",
                    "duration": "15m",
                    "size": "1.4 GB",
                    "status": "Uploaded",
                },
                {
                    "id": "bag2",
                    "name": "mission-1030.bag",
                    "duration": "26m",
                    "size": "2.7 GB",
                    "status": "Processing",
                },
            ],
            "robot_settings": {
                "autopilot": True,
                "safeMode": True,
                "remoteDiagnostics": False,
                "pathOptimization": True,
            },
            "account": {
                "fullName": "Alex Operator",
                "email": "alex.operator@example.com",
                "team": "Ops",
                "shift": "06:00–14:00",
            },
            "appearance": {
                "theme": "system",
                "accentColor": "#0b74d1",
            },
            "security": {
                "twoFactor": True,
                "autoLock": True,
                "anomalyAlerts": True,
            },
            "security_events": [
                {
                    "id": "sec1",
                    "ts": "09:44",
                    "actor": "ops-admin",
                    "action": "API token created",
                    "context": "Main console",
                },
                {
                    "id": "sec2",
                    "ts": "08:12",
                    "actor": "robot-01",
                    "action": "Cert renewed",
                    "context": "Device",
                },
            ],
            "integrations": [
                {
                    "id": "rest",
                    "name": "REST API",
                    "status": "Connected",
                    "description": "Push missions from MES",
                },
                {
                    "id": "slack",
                    "name": "Slack Bot",
                    "status": "Disconnected",
                    "description": "Alerts to #robot-ops",
                },
                {
                    "id": "grafana",
                    "name": "Grafana",
                    "status": "Connected",
                    "description": "Telemetry dashboards",
                },
            ],
            "chat_messages": [
                {
                    "id": "msg1",
                    "text": "Hello! I'm your robot assistant. How can I help you today?",
                    "sender": "robot",
                    "timestamp": _utc_ts(),
                    "status": "Delivered",
                }
            ],
            "stats": {
                "overview": {
                    "totalKm": 182.4,
                    "missionsCompleted": 47,
                    "avgSpeed": 1.8,
                    "operatingHours": 326,
                    "deltaKm": 4.3,
                    "missionSuccessRate": 98,
                },
                "missionTrend": [
                    {"label": "Mon", "completed": 5, "incidents": 0},
                    {"label": "Tue", "completed": 7, "incidents": 1},
                    {"label": "Wed", "completed": 6, "incidents": 0},
                    {"label": "Thu", "completed": 8, "incidents": 1},
                    {"label": "Fri", "completed": 9, "incidents": 0},
                ],
                "monthlyMovement": [
                    {"month": "Jan", "km": 118},
                    {"month": "Feb", "km": 142},
                    {"month": "Mar", "km": 131},
                    {"month": "Apr", "km": 155},
                    {"month": "May", "km": 162},
                    {"month": "Jun", "km": 174},
                ],
                "batterySeries": [
                    {"time": "08:00", "voltage": 48.2, "power": 182},
                    {"time": "09:00", "voltage": 47.8, "power": 176},
                    {"time": "10:00", "voltage": 47.4, "power": 171},
                    {"time": "11:00", "voltage": 46.9, "power": 168},
                    {"time": "12:00", "voltage": 47.1, "power": 170},
                    {"time": "13:00", "voltage": 46.7, "power": 166},
                    {"time": "14:00", "voltage": 46.3, "power": 164},
                ],
                "batteryStatus": {
                    "packVoltage": 46.9,
                    "packCurrent": 38.2,
                    "stateOfCharge": 78,
                    "temperature": "32°C",
                    "cycles": 412,
                    "health": "Good",
                    "cells": [
                        {"id": "Cell A", "voltage": 3.9},
                        {"id": "Cell B", "voltage": 3.9},
                        {"id": "Cell C", "voltage": 3.88},
                        {"id": "Cell D", "voltage": 3.87},
                    ],
                },
                "turns": {"left": 132, "right": 148},
            },
        }

    def _collection_index(self, bucket: str, item_id: str) -> Tuple[int, Dict[str, Any]]:
        for idx, item in enumerate(self._state[bucket]):
            if item["id"] == item_id:
                return idx, item
        raise KeyError(item_id)

    # ------------------------------------------------------------------
    # Maps
    def list_maps(self) -> List[Dict[str, Any]]:
        with self._lock:
            return _clone(self._state["maps"])

    def create_map(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            new_map = {
                "id": payload.get("id") or self._new_id("map"),
                "name": payload.get("name", "Unnamed Map"),
                "createdBy": payload.get("createdBy", "Unknown"),
                "status": payload.get("status", "Draft"),
                "previewImage": payload.get("previewImage", ""),
                "createdAt": payload.get("createdAt", _utc_ts()),
            }
            self._state["maps"].append(new_map)
            return _clone(new_map)

    def get_map(self, map_id: str) -> Dict[str, Any]:
        with self._lock:
            _, item = self._collection_index("maps", map_id)
            return _clone(item)

    def update_map(self, map_id: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            idx, item = self._collection_index("maps", map_id)
            updated = {**item, **payload, "id": map_id}
            self._state["maps"][idx] = updated
            return _clone(updated)

    def delete_map(self, map_id: str) -> None:
        with self._lock:
            idx, _ = self._collection_index("maps", map_id)
            self._state["maps"].pop(idx)

    def mark_map_loaded(self, map_id: str) -> Dict[str, Any]:
        with self._lock:
            idx, item = self._collection_index("maps", map_id)
            updated = {**item, "lastLoadedAt": _utc_ts()}
            self._state["maps"][idx] = updated
            return _clone(updated)

    # ------------------------------------------------------------------
    # Zones
    def list_zones(self) -> List[Dict[str, Any]]:
        with self._lock:
            return _clone(self._state["zones"])

    def create_zone(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            zone = {
                "id": payload.get("id") or self._new_id("zone"),
                "name": payload.get("name", "New Zone"),
                "category": payload.get("category", "Safe"),
                "geometry": payload.get("geometry", ""),
                "active": bool(payload.get("active", True)),
                "createdAt": payload.get("createdAt", _utc_ts()),
            }
            self._state["zones"].append(zone)
            return _clone(zone)

    def update_zone(self, zone_id: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            idx, item = self._collection_index("zones", zone_id)
            updated = {**item, **payload, "id": zone_id}
            self._state["zones"][idx] = updated
            return _clone(updated)

    def delete_zone(self, zone_id: str) -> None:
        with self._lock:
            idx, _ = self._collection_index("zones", zone_id)
            self._state["zones"].pop(idx)

    # ------------------------------------------------------------------
    # Waypoints
    def list_waypoints(self) -> List[Dict[str, Any]]:
        with self._lock:
            return _clone(self._state["waypoints"])

    def create_waypoint(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            wp = {
                "id": payload.get("id") or self._new_id("wp"),
                "name": payload.get("name", "Waypoint"),
                "category": payload.get("category", "Nav"),
                "geom": payload.get("geom", "Point(0 0)"),
                "active": bool(payload.get("active", True)),
                "notes": payload.get("notes", ""),
                "createdAt": payload.get("createdAt", _utc_ts()),
            }
            self._state["waypoints"].append(wp)
            return _clone(wp)

    def update_waypoint(self, waypoint_id: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            idx, item = self._collection_index("waypoints", waypoint_id)
            updated = {**item, **payload, "id": waypoint_id}
            self._state["waypoints"][idx] = updated
            return _clone(updated)

    def delete_waypoint(self, waypoint_id: str) -> None:
        with self._lock:
            idx, _ = self._collection_index("waypoints", waypoint_id)
            self._state["waypoints"].pop(idx)

    # ------------------------------------------------------------------
    # Missions
    def list_missions(self) -> List[Dict[str, Any]]:
        with self._lock:
            return _clone(self._state["missions"])

    def create_mission(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            mission = {
                "id": payload.get("id") or self._new_id("mission"),
                "name": payload.get("name", "New Mission"),
                "owner": payload.get("owner", "Unknown"),
                "status": payload.get("status", "Draft"),
                "notes": payload.get("notes", ""),
                "createdAt": payload.get("createdAt", _utc_ts()),
            }
            self._state["missions"].append(mission)
            return _clone(mission)

    def update_mission(self, mission_id: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            idx, item = self._collection_index("missions", mission_id)
            updated = {**item, **payload, "id": mission_id}
            self._state["missions"][idx] = updated
            return _clone(updated)

    def delete_mission(self, mission_id: str) -> None:
        with self._lock:
            idx, _ = self._collection_index("missions", mission_id)
            self._state["missions"].pop(idx)

    def initiate_mission(self, mission_id: str) -> Dict[str, Any]:
        with self._lock:
            idx, item = self._collection_index("missions", mission_id)
            updated = {**item, "status": "Running", "startedAt": _utc_ts()}
            self._state["missions"][idx] = updated
            return _clone(updated)

    # ------------------------------------------------------------------
    # Users
    def list_users(self) -> List[Dict[str, Any]]:
        with self._lock:
            return _clone(self._state["users"])

    def create_user(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            user = {
                "id": payload.get("id") or self._new_id("user"),
                "name": payload.get("name", "New User"),
                "email": payload.get("email", ""),
                "role": payload.get("role", "Operator"),
                "status": payload.get("status", "Pending"),
                "createdBy": payload.get("createdBy", "System"),
                "createdAt": payload.get("createdAt", _utc_ts()),
            }
            self._state["users"].append(user)
            return _clone(user)

    def update_user(self, user_id: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            idx, item = self._collection_index("users", user_id)
            updated = {**item, **payload, "id": user_id}
            self._state["users"][idx] = updated
            return _clone(updated)

    def delete_user(self, user_id: str) -> None:
        with self._lock:
            idx, _ = self._collection_index("users", user_id)
            self._state["users"].pop(idx)

    def get_user(self, user_id: str) -> Dict[str, Any]:
        with self._lock:
            _, item = self._collection_index("users", user_id)
            return _clone(item)

    def reset_user_password(self, user_id: str) -> Dict[str, Any]:
        with self._lock:
            idx, item = self._collection_index("users", user_id)
            updated = {**item, "lastPasswordReset": _utc_ts()}
            self._state["users"][idx] = updated
            return _clone(updated)

    # ------------------------------------------------------------------
    # Monitor data
    def get_analytics(self) -> Dict[str, Any]:
        with self._lock:
            return _clone(self._state["analytics"])

    def add_analytics_alert(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            alert = {
                "id": payload.get("id") or self._new_id("alert"),
                "title": payload.get("title", "Alert"),
                "detail": payload.get("detail", ""),
            }
            self._state["analytics"]["alerts"].insert(0, alert)
            return _clone(alert)

    def update_analytics_summary(self, label: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            for entry in self._state["analytics"]["summary"]:
                if entry["label"].lower() == label.lower():
                    entry.update(payload)
                    return _clone(entry)
        raise KeyError(label)

    def set_analytics_series(self, series: List[int]) -> List[int]:
        with self._lock:
            self._state["analytics"]["series"] = list(series)
            return _clone(self._state["analytics"]["series"])

    def get_diagnostics(self) -> List[Dict[str, Any]]:
        with self._lock:
            return _clone(self._state["diagnostics"])

    def update_diagnostic(self, diag_id: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            idx, item = self._collection_index("diagnostics", diag_id)
            updated = {**item, **payload, "id": diag_id}
            self._state["diagnostics"][idx] = updated
            return _clone(updated)

    def get_logs(self) -> List[Dict[str, Any]]:
        with self._lock:
            return _clone(self._state["logs"])

    def add_log(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            log = {
                "id": payload.get("id") or self._new_id("log"),
                "ts": payload.get("ts", _utc_ts()),
                "system": payload.get("system", "System"),
                "message": payload.get("message", ""),
                "level": payload.get("level", "info"),
            }
            self._state["logs"].insert(0, log)
            return _clone(log)

    def get_mission_history(self) -> List[Dict[str, Any]]:
        with self._lock:
            return _clone(self._state["mission_logs"])

    def add_mission_history(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            entry = {
                "id": payload.get("id") or self._new_id("mh"),
                "mission": payload.get("mission", "Unnamed"),
                "window": payload.get("window", ""),
                "outcome": payload.get("outcome", "Completed"),
                "notes": payload.get("notes", ""),
            }
            self._state["mission_logs"].insert(0, entry)
            return _clone(entry)

    def get_robot_bags(self) -> List[Dict[str, Any]]:
        with self._lock:
            return _clone(self._state["robot_bags"])

    def add_robot_bag(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            bag = {
                "id": payload.get("id") or self._new_id("bag"),
                "name": payload.get("name", "recording.bag"),
                "duration": payload.get("duration", "0m"),
                "size": payload.get("size", "0 MB"),
                "status": payload.get("status", "Uploaded"),
            }
            self._state["robot_bags"].insert(0, bag)
            return _clone(bag)

    # ------------------------------------------------------------------
    # Settings
    def get_robot_settings(self) -> Dict[str, Any]:
        with self._lock:
            return _clone(self._state["robot_settings"])

    def update_robot_settings(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            self._state["robot_settings"].update(payload)
            return _clone(self._state["robot_settings"])

    def get_account(self) -> Dict[str, Any]:
        with self._lock:
            return _clone(self._state["account"])

    def update_account(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            self._state["account"].update(payload)
            self._state["account"]["updatedAt"] = _utc_ts()
            return _clone(self._state["account"])

    def get_appearance(self) -> Dict[str, Any]:
        with self._lock:
            return _clone(self._state["appearance"])

    def update_appearance(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            self._state["appearance"].update(payload)
            return _clone(self._state["appearance"])

    def get_security(self) -> Dict[str, Any]:
        with self._lock:
            return _clone(self._state["security"])

    def update_security(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            self._state["security"].update(payload)
            return _clone(self._state["security"])

    def list_security_events(self) -> List[Dict[str, Any]]:
        with self._lock:
            return _clone(self._state["security_events"])

    def add_security_event(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            entry = {
                "id": payload.get("id") or self._new_id("sec"),
                "ts": payload.get("ts", _utc_ts()),
                "actor": payload.get("actor", "system"),
                "action": payload.get("action", "Updated"),
                "context": payload.get("context", "Console"),
            }
            self._state["security_events"].insert(0, entry)
            return _clone(entry)

    def list_integrations(self) -> List[Dict[str, Any]]:
        with self._lock:
            return _clone(self._state["integrations"])

    def add_integration(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            item = {
                "id": payload.get("id") or self._new_id("integration"),
                "name": payload.get("name", "Integration"),
                "status": payload.get("status", "Disconnected"),
                "description": payload.get("description", ""),
            }
            self._state["integrations"].append(item)
            return _clone(item)

    def update_integration(self, integration_id: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            idx, item = self._collection_index("integrations", integration_id)
            updated = {**item, **payload, "id": integration_id}
            self._state["integrations"][idx] = updated
            return _clone(updated)

    # ------------------------------------------------------------------
    # Chat
    def list_chat_messages(self) -> List[Dict[str, Any]]:
        with self._lock:
            return _clone(self._state["chat_messages"])

    def add_chat_message(self, sender: str, text: str) -> Dict[str, Any]:
        with self._lock:
            message = {
                "id": self._new_id("msg"),
                "text": text,
                "sender": sender,
                "timestamp": _utc_ts(),
                "status": "Delivered" if sender == "robot" else "Sent",
            }
            self._state["chat_messages"].append(message)
            return _clone(message)

    # ------------------------------------------------------------------
    def get_stats(self) -> Dict[str, Any]:
        with self._lock:
            return _clone(self._state["stats"])


__all__ = ["FrontendDataStore"]
