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
from typing import Any, Dict, List, Optional, Tuple
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

    def _new_object_id(self) -> str:
        """Generate a 24-char hex string similar to MongoDB ObjectId."""
        return uuid4().hex[:24]

    def _current_user_info(self) -> Dict[str, str]:
        """Lightweight helper to pull the mock 'logged-in' user profile."""
        account = self._state.get("account", {})
        return {
            "name": account.get("fullName", "System"),
            "email": account.get("email", ""),
        }

    def _normalize_mission(self, mission: Dict[str, Any]) -> Dict[str, Any]:
        """Ensure missions always include ownership metadata derived from the user profile."""
        current_user = self._current_user_info()
        raw_created_by = mission.get("createdBy")
        created_by = (
            raw_created_by
            if isinstance(raw_created_by, dict)
            else {"name": current_user["name"], "email": current_user["email"]}
        )
        owner_email = (
            mission.get("email")
            or mission.get("ownerEmail")
            or created_by.get("email")
            or current_user["email"]
        )
        normalized = {
            **mission,
            "owner": mission.get("owner") or created_by.get("name") or current_user["name"],
            "email": owner_email,
            "createdBy": created_by,
            "createdAt": mission.get("createdAt") or _utc_ts(),
            "status": mission.get("status") or ("Active" if mission.get("isActive", True) else "Inactive"),
        }
        return normalized

    def _default_state(self) -> Dict[str, Any]:
        return {
            "maps": [
                {
                    "id": "63660bcc15d4c31b0f42afb3",
                    "_id": "63660bcc15d4c31b0f42afb3",
                    "name": "CFL_GF",
                    "createdBy": "CNDE IITM",
                    "status": "Active",
                    "isActive": True,
                    "createdAt": "2025-11-12T08:30:00Z",
                    "previewImage": "",
                    "properties": {
                        "origin": {
                            "position": {"x": 0.005012, "y": 0.005012, "z": 0.005012},
                            "orientation": {"x": 0.005012, "y": 0.005012, "z": 0.005012, "w": 0.005012},
                        },
                        "width": 420,
                        "height": 1260,
                        "resolution": 0.005012,
                        "previewUrl": "",
                    },
                },
                {
                    "id": "6365025662a66cbf3123562e",
                    "_id": "6365025662a66cbf3123562e",
                    "name": "shobot_arena",
                    "createdBy": "CNDE IITM",
                    "status": "",
                    "isActive": False,
                    "createdAt": "2025-11-10T05:12:00Z",
                    "previewImage": "/images/maps/shobot_arena.png",
                    "properties": {
                        "origin": {
                            "position": {"x": 0.005012, "y": 0.005012, "z": 0.005012},
                            "orientation": {"x": 0.005012, "y": 0.005012, "z": 0.005012, "w": 0.005012},
                        },
                        "width": 420,
                        "height": 1260,
                        "resolution": 0.005012,
                        "previewUrl": "https://localhost:5000/previewMap1",
                    },
                },
                {
                    "id": "63621a320e8f3ea6b22dd668",
                    "_id": "63621a320e8f3ea6b22dd668",
                    "name": "shobot_arena2",
                    "createdBy": "CNDE IITM",
                    "status": "",
                    "isActive": True,
                    "createdAt": "2025-11-09T18:44:00Z",
                    "previewImage": "/images/maps/shobot_arena2.png",
                    "properties": {
                        "origin": {
                            "position": {"x": 0.005012, "y": 0.005012, "z": 0.005012},
                            "orientation": {"x": 0.005012, "y": 0.005012, "z": 0.005012, "w": 0.005012},
                        },
                        "width": 420,
                        "height": 1260,
                        "resolution": 0.005012,
                        "previewUrl": "https://localhost:5000/previewMap1",
                    },
                },
                {
                    "id": "6360b98e80dccb699a18fbd6",
                    "_id": "6360b98e80dccb699a18fbd6",
                    "name": "Zones",
                    "createdBy": "CNDE IITM",
                    "status": "",
                    "isActive": False,
                    "createdAt": "2025-11-09T18:40:00Z",
                    "previewImage": "",
                    "properties": {
                        "origin": {
                            "position": {"x": 0.005015, "y": 0.005012, "z": 0.005012},
                            "orientation": {"x": 0.005012, "y": 0.005012, "z": 0.005012, "w": 0.005012},
                        },
                        "width": 420,
                        "height": 1260,
                        "resolution": 0.005012,
                        "previewUrl": "https://localhost:5000/previewMap2",
                    },
                },
                {
                    "id": "63660bcc15d4c31b0f42afb5",
                    "_id": "63660bcc15d4c31b0f42afb5",
                    "name": "Waypoints",
                    "createdBy": "CNDE IITM",
                    "status": "",
                    "isActive": False,
                    "createdAt": "2025-11-09T18:41:00Z",
                    "previewImage": "",
                    "properties": {
                        "origin": {
                            "position": {"x": 0.005012, "y": 0.005012, "z": 0.005012},
                            "orientation": {"x": 0.005012, "y": 0.005012, "z": 0.005012, "w": 0.005012},
                        },
                        "width": 420,
                        "height": 1260,
                        "resolution": 0.005012,
                        "previewUrl": "",
                    },
                },
                {
                    "id": "6365025662a66cbf31235630",
                    "_id": "6365025662a66cbf31235630",
                    "name": "Users",
                    "createdBy": "CNDE IITM",
                    "status": "",
                    "isActive": False,
                    "createdAt": "2025-11-09T18:42:00Z",
                    "previewImage": "",
                    "properties": {
                        "origin": {
                            "position": {"x": 0.005012, "y": 0.005012, "z": 0.005012},
                            "orientation": {"x": 0.005012, "y": 0.005012, "z": 0.005012, "w": 0.005012},
                        },
                        "width": 420,
                        "height": 1260,
                        "resolution": 0.005012,
                        "previewUrl": "",
                    },
                },
                {
                    "id": "6371a9ca8a134c1047891d21",
                    "_id": "6371a9ca8a134c1047891d21",
                    "name": "WAKANDA",
                    "createdBy": "6371a6778a134c1047891cfd",
                    "status": "",
                    "isActive": False,
                    "createdAt": "2022-11-14T02:36:58.827Z",
                    "previewImage": "https://localhost:5000/previewMap1",
                    "properties": {
                        "origin": {
                            "position": {"x": 0.005012, "y": 0.005012, "z": 0.005012},
                            "orientation": {"x": 0.005012, "y": 0.005012, "z": 0.005012, "w": 0.005012},
                            "_id": "6371a9ca8a134c1047891d23",
                        },
                        "width": 420,
                        "height": 1260,
                        "resolution": 0.005012,
                        "previewUrl": "https://localhost:5000/previewMap1",
                        "_id": "6371a9ca8a134c1047891d22",
                    },
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
                    "id": "wp_63660bcc15d4c31b0f42aaa1",
                    "_id": "63660bcc15d4c31b0f42aaa1",
                    "name": "Dock A",
                    "category": "CHARGE",
                    "isActive": True,
                    "createdBy": "CNDE IITM",
                    "createdAt": "2025-11-17T09:00:00Z",
                    "notes": "Primary dock",
                    "coordinate": {
                        "position": {"x": 1.0, "y": 2.0, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                },
                {
                    "id": "wp_6365025662a66cbf31230001",
                    "_id": "6365025662a66cbf31230001",
                    "name": "Zone A Entry",
                    "category": "NAV",
                    "isActive": True,
                    "createdBy": "CNDE IITM",
                    "createdAt": "2025-11-17T09:05:00Z",
                    "notes": "Entry to Zone A",
                    "coordinate": {
                        "position": {"x": 5.4, "y": -1.2, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                },
                {
                    "id": "wp_63621a320e8f3ea6b22dd001",
                    "_id": "63621a320e8f3ea6b22dd001",
                    "name": "Inspection P1",
                    "category": "INSPECT",
                    "isActive": False,
                    "createdBy": "CNDE IITM",
                    "createdAt": "2025-11-16T14:20:00Z",
                    "notes": "Manual inspection point",
                    "coordinate": {
                        "position": {"x": -2.0, "y": 3.5, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                },
            ],
            "missions": [
                {
                    "id": "635acf02c53974c5497949da",
                    "_id": "635acf02c53974c5497949da",
                    "name": "Wakanda",
                    "email": "wakanda@anscer.com",
                    "role": "admin",
                    "createdBy": None,
                    "createdAt": "2022-10-27T18:33:38.718Z",
                    "updatedAt": "2022-11-12T06:37:30.021Z",
                    "__v": 0,
                    "updatedBy": {
                        "_id": "635d2b7628b2f98a00580803",
                        "name": "Ashwanee Kumar Gupta",
                        "email": "ashwanee@anscer.com",
                    },
                    "isActive": True,
                },
                {
                    "id": "635d47031975c4da01c1f498",
                    "_id": "635d47031975c4da01c1f498",
                    "name": "abcd abcd",
                    "email": "abcd@anscer.com",
                    "role": "admin",
                    "createdBy": {
                        "_id": "635acf02c53974c5497949da",
                        "name": "Wakanda",
                        "email": "wakanda@anscer.com",
                    },
                    "createdAt": "2022-10-29T15:30:11.125Z",
                    "updatedAt": "2022-11-07T11:39:15.677Z",
                    "__v": 0,
                    "updatedBy": {
                        "_id": "635d2b7628b2f98a00580803",
                        "name": "Ashwanee Kumar Gupta",
                        "email": "ashwanee@anscer.com",
                    },
                    "isActive": False,
                },
                {
                    "id": "6368ed66fe89e2cbc83fa9bd",
                    "_id": "6368ed66fe89e2cbc83fa9bd",
                    "name": "Ashwanee",
                    "email": "ashwanee@gmail.com",
                    "isActive": True,
                    "role": "admin",
                    "createdBy": {
                        "_id": "635d2b7628b2f98a00580803",
                        "name": "Ashwanee Kumar Gupta",
                        "email": "ashwanee@anscer.com",
                    },
                    "createdAt": "2022-11-07T11:35:02.754Z",
                    "updatedAt": "2022-11-12T10:20:57.006Z",
                    "__v": 0,
                    "updatedBy": {
                        "_id": "635d2b7628b2f98a00580803",
                        "name": "Ashwanee Kumar Gupta",
                        "email": "ashwanee@anscer.com",
                    },
                },
                {
                    "id": "636a883208adc339b8a8c0c8",
                    "_id": "636a883208adc339b8a8c0c8",
                    "name": "Ashwanee",
                    "email": "ashwanee2001gupta@gmail.com",
                    "isActive": True,
                    "role": "admin",
                    "createdBy": {
                        "_id": "635d2b7628b2f98a00580803",
                        "name": "Ashwanee Kumar Gupta",
                        "email": "ashwanee@anscer.com",
                    },
                    "createdAt": "2022-11-08T16:47:46.262Z",
                    "updatedAt": "2022-11-08T16:47:46.262Z",
                    "__v": 0,
                },
                {
                    "id": "636f1212dcf1275db2c6afba",
                    "_id": "636f1212dcf1275db2c6afba",
                    "name": "WhiteWolf",
                    "email": "whitewolf@anscer.com",
                    "isActive": True,
                    "role": "admin",
                    "createdBy": {
                        "_id": "635d2b7628b2f98a00580803",
                        "name": "Ashwanee Kumar Gupta",
                        "email": "ashwanee@anscer.com",
                    },
                    "createdAt": "2022-11-12T03:25:06.626Z",
                    "updatedAt": "2022-11-12T03:25:06.626Z",
                    "__v": 0,
                },
                {
                    "id": "635ad9430c89aac7fd5b8968",
                    "_id": "635ad9430c89aac7fd5b8968",
                    "name": "Test User 2",
                    "email": "test4@anscer.com",
                    "role": "developer",
                    "createdBy": {
                        "_id": "635acf02c53974c5497949da",
                        "name": "Wakanda",
                        "email": "wakanda@anscer.com",
                    },
                    "createdAt": "2022-10-27T19:17:23.934Z",
                    "updatedAt": "2022-10-29T14:00:08.081Z",
                    "__v": 0,
                    "updatedBy": None,
                    "isActive": True,
                },
                {
                    "id": "635d2b7628b2f98a00580803",
                    "_id": "635d2b7628b2f98a00580803",
                    "name": "Ashwanee Kumar Gupta",
                    "email": "ashwanee@anscer.com",
                    "role": "developer",
                    "createdAt": "2022-10-29T13:32:38.770Z",
                    "updatedAt": "2022-10-29T13:32:38.770Z",
                    "__v": 0,
                    "isActive": True,
                },
                {
                    "id": "636a6c7c08adc339b8a80553",
                    "_id": "636a6c7c08adc339b8a80553",
                    "name": "Raghu V",
                    "email": "raghu@anscer.com",
                    "isActive": True,
                    "role": "developer",
                    "createdBy": {
                        "_id": "635d2b7628b2f98a00580803",
                        "name": "Ashwanee Kumar Gupta",
                        "email": "ashwanee@anscer.com",
                    },
                    "createdAt": "2022-11-08T14:49:32.790Z",
                    "updatedAt": "2022-11-08T14:49:32.790Z",
                    "__v": 0,
                },
                {
                    "id": "635ac64ffec0c050435623ee",
                    "_id": "635ac64ffec0c050435623ee",
                    "name": "Test User 2",
                    "email": "test2@anscer.com",
                    "role": "user",
                    "createdBy": None,
                    "createdAt": "2022-10-27T17:56:31.223Z",
                    "updatedAt": "2022-10-29T18:01:57.727Z",
                    "__v": 0,
                    "updatedBy": {
                        "_id": "635acf02c53974c5497949da",
                        "name": "Wakanda",
                        "email": "wakanda@anscer.com",
                    },
                    "isActive": True,
                },
                {
                    "id": "635ad24655bc671e40735dc6",
                    "_id": "635ad24655bc671e40735dc6",
                    "name": "BlaBla Bla",
                    "email": "blablabla@anscer.com",
                    "role": "user",
                    "createdBy": {
                        "_id": "635acf02c53974c5497949da",
                        "name": "Wakanda",
                        "email": "wakanda@anscer.com",
                    },
                    "createdAt": "2022-10-27T18:47:34.208Z",
                    "updatedAt": "2022-10-29T14:24:52.115Z",
                    "__v": 0,
                    "updatedBy": None,
                    "isActive": True,
                },
                {
                    "id": "636b4c7b08adc339b8a8c28d",
                    "_id": "636b4c7b08adc339b8a8c28d",
                    "name": "Website",
                    "email": "website@gmail.com",
                    "isActive": True,
                    "role": "user",
                    "createdBy": {
                        "_id": "635d2b7628b2f98a00580803",
                        "name": "Ashwanee Kumar Gupta",
                        "email": "ashwanee@anscer.com",
                    },
                    "createdAt": "2022-11-09T06:45:15.565Z",
                    "updatedAt": "2022-11-09T06:45:15.565Z",
                    "__v": 0,
                },
                {
                    "id": "636f73badcf1275db2c6b022",
                    "_id": "636f73badcf1275db2c6b022",
                    "name": "New User",
                    "email": "newuser@abc.com",
                    "isActive": True,
                    "role": "user",
                    "createdBy": {
                        "_id": "635d2b7628b2f98a00580803",
                        "name": "Ashwanee Kumar Gupta",
                        "email": "ashwanee@anscer.com",
                    },
                    "createdAt": "2022-11-12T10:21:46.946Z",
                    "updatedAt": "2022-11-12T10:21:46.946Z",
                    "__v": 0,
                },
                {
                    "id": "6371a6778a134c1047891cfd",
                    "_id": "6371a6778a134c1047891cfd",
                    "name": "Ashwanee",
                    "email": "ashwanee@abc.com",
                    "isActive": True,
                    "role": "user",
                    "createdBy": {
                        "_id": "635d2b7628b2f98a00580803",
                        "name": "Ashwanee Kumar Gupta",
                        "email": "ashwanee@anscer.com",
                    },
                    "createdAt": "2022-11-14T02:22:47.669Z",
                    "updatedAt": "2022-11-14T02:22:47.669Z",
                    "__v": 0,
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
            object_id = payload.get("_id") or self._new_object_id()
            new_map = {
                "id": payload.get("id") or object_id,
                "_id": object_id,
                "name": payload.get("name", "Unnamed Map"),
                "createdBy": payload.get("createdBy", "CNDE IITM"),
                "status": payload.get("status", "Draft"),
                "isActive": payload.get("isActive", False),
                "previewImage": payload.get("previewImage", ""),
                "createdAt": payload.get("createdAt", _utc_ts()),
                "properties": payload.get(
                    "properties",
                    {
                        "origin": {
                            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                        },
                        "width": 0,
                        "height": 0,
                        "resolution": 0.05,
                        "previewUrl": "",
                    },
                ),
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
            object_id = payload.get("_id") or self._new_object_id()
            zone = {
                "id": payload.get("id") or object_id,
                "_id": object_id,
                "name": payload.get("name", "New Zone"),
                "category": payload.get("category", "restricted"),
                "geometry": payload.get("geometry", {}),
                "properties": payload.get("properties", {}),
                "mapId": payload.get("mapId"),
                "isActive": bool(payload.get("isActive", True)),
                "createdBy": payload.get("createdBy", "CNDE IITM"),
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
            object_id = payload.get("_id") or self._new_object_id()
            wp = {
                "id": payload.get("id") or object_id,
                "_id": object_id,
                "name": payload.get("name", "Waypoint"),
                "category": payload.get("category", "NAV"),
                "coordinate": payload.get(
                    "coordinate",
                    {
                        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                ),
                "mapId": payload.get("mapId"),
                "isActive": bool(payload.get("isActive", True)),
                "notes": payload.get("notes", ""),
                "createdBy": payload.get("createdBy", "CNDE IITM"),
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
            missions = [_clone(self._normalize_mission(m)) for m in self._state["missions"]]
            return missions

    def create_mission(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            current_user = self._current_user_info()
            created_by = payload.get("createdBy") or {
                "name": current_user["name"],
                "email": current_user["email"],
            }
            owner_email = payload.get("email") or payload.get("ownerEmail") or created_by.get("email")
            object_id = payload.get("_id") or self._new_object_id()
            mission = {
                "id": payload.get("id") or object_id,
                "_id": object_id,
                "name": payload.get("name", "New Mission"),
                "owner": payload.get("owner") or created_by.get("name") or current_user["name"],
                "email": owner_email,
                "createdBy": created_by,
                "status": payload.get("status", "Draft"),
                "notes": payload.get("notes", ""),
                "mapId": payload.get("mapId"),
                "waypoints": payload.get("waypoints", []),
                "isActive": bool(payload.get("isActive", True)),
                "createdAt": payload.get("createdAt", _utc_ts()),
            }
            normalized = self._normalize_mission(mission)
            self._state["missions"].append(normalized)
            return _clone(normalized)

    def update_mission(self, mission_id: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            idx, item = self._collection_index("missions", mission_id)
            updated = self._normalize_mission({**item, **payload, "id": mission_id})
            self._state["missions"][idx] = updated
            return _clone(updated)

    def delete_mission(self, mission_id: str) -> None:
        with self._lock:
            idx, _ = self._collection_index("missions", mission_id)
            self._state["missions"].pop(idx)

    def initiate_mission(self, mission_id: str) -> Dict[str, Any]:
        with self._lock:
            idx, item = self._collection_index("missions", mission_id)
            updated = self._normalize_mission(
                {**item, "status": "Running", "startedAt": _utc_ts()}
            )
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

    def add_chat_message(self, sender: str, text: str, metadata: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        with self._lock:
            message = {
                "id": self._new_id("msg"),
                "text": text,
                "sender": sender,
                "timestamp": _utc_ts(),
                "status": "Delivered" if sender == "robot" else "Sent",
            }
            if metadata:
                message["metadata"] = metadata
            self._state["chat_messages"].append(message)
            return _clone(message)

    # ------------------------------------------------------------------
    def get_stats(self) -> Dict[str, Any]:
        with self._lock:
            return _clone(self._state["stats"])


__all__ = ["FrontendDataStore"]
