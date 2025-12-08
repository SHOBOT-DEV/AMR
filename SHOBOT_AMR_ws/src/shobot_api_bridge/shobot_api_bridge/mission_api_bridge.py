#!/usr/bin/env python3
"""
Mission API bridge:
- Polls /api/v1/missions for new missions
- Publishes them onto the ROS2 mission queue topic
- Updates API mission status to reflect dispatch

Environment:
  API_URL    (default: http://localhost:5000/api/v1)
  API_TOKEN  (Bearer token; optional if API is unauthenticated)
"""

import json
import os
from typing import Dict, List, Optional, Set

import rclpy
import requests
from rclpy.node import Node
from std_msgs.msg import String


class MissionApiBridge(Node):
    """Pull missions from the HTTP API and push them to the ROS mission queue."""

    def __init__(self):
        super().__init__("mission_api_bridge")
        self.declare_parameter("mission_queue_topic", "/mission_queue")
        self.declare_parameter("poll_interval", 2.0)
        self.declare_parameter("api_url", os.environ.get("API_URL", "http://localhost:5000/api/v1"))
        self.declare_parameter("api_token", os.environ.get("API_TOKEN", ""))

        mission_queue_topic = self.get_parameter("mission_queue_topic").value
        poll_interval = float(self.get_parameter("poll_interval").value)
        self.api_url = str(self.get_parameter("api_url").value).rstrip("/")
        self.api_token = str(self.get_parameter("api_token").value)

        self.queue_pub = self.create_publisher(String, mission_queue_topic, 10)
        self.dispatched: Set[str] = set()

        self.timer = self.create_timer(poll_interval, self._poll_and_dispatch)
        self.get_logger().info(
            f"Mission API bridge started (poll {poll_interval}s, topic: {mission_queue_topic}, api: {self.api_url})"
        )

    # ------------------- API helpers -------------------
    def _headers(self) -> Dict[str, str]:
        headers = {"Accept": "application/json", "Content-Type": "application/json"}
        if self.api_token:
            headers["Authorization"] = f"Bearer {self.api_token}"
        return headers

    def _get_missions(self) -> List[Dict]:
        try:
            res = requests.get(f"{self.api_url}/missions", headers=self._headers(), timeout=5)
            res.raise_for_status()
            payload = res.json()
            return payload.get("items", [])
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Mission fetch failed: {exc}")
            return []

    def _update_status(self, mission_id: str, status: str, notes: Optional[str] = None) -> None:
        body = {"status": status}
        if notes:
            body["notes"] = notes
        try:
            requests.put(
                f"{self.api_url}/missions/{mission_id}",
                headers=self._headers(),
                data=json.dumps(body),
                timeout=5,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Status update failed for {mission_id}: {exc}")

    # ------------------- Mission handling -------------------
    def _poll_and_dispatch(self):
        missions = self._get_missions()
        for mission in missions:
            mission_id = mission.get("id") or mission.get("_id")
            if not mission_id or mission_id in self.dispatched:
                continue

            # Only dispatch missions marked as Running/Queued/Active
            status = (mission.get("status") or "").lower()
            if status not in {"running", "queued", "active"}:
                continue

            payload = self._mission_to_queue_payload(mission)
            if not payload["missions"]:
                self.get_logger().warn(f"Mission {mission_id} has no waypoints/poses; skipping dispatch.")
                continue

            self.queue_pub.publish(String(data=json.dumps(payload)))
            self.dispatched.add(mission_id)
            self.get_logger().info(f"Dispatched mission {mission_id} to queue with {len(payload['missions'])} steps.")
            # Mark as queued/dispatched in API so it isn't picked again if restarted quickly.
            self._update_status(mission_id, "Queued", notes="Dispatched to ROS mission queue")

    def _mission_to_queue_payload(self, mission: Dict) -> Dict[str, object]:
        """Convert API mission into mission_control format."""
        steps: List[Dict[str, object]] = []
        for wp in mission.get("waypoints", []):
            coord = wp.get("coordinate") or {}
            pos = coord.get("position", {})
            orient = coord.get("orientation", {})
            if not pos:
                continue
            step = {
                "pose": {
                    "frame_id": coord.get("frame_id", "map"),
                    "position": {
                        "x": float(pos.get("x", 0.0)),
                        "y": float(pos.get("y", 0.0)),
                        "z": float(pos.get("z", 0.0)),
                    },
                    "orientation": {
                        "x": float(orient.get("x", 0.0)),
                        "y": float(orient.get("y", 0.0)),
                        "z": float(orient.get("z", 0.0)),
                        "w": float(orient.get("w", 1.0)),
                    },
                },
                "dock": bool(wp.get("dock", False)),
            }
            steps.append(step)

        return {
            "mission_id": mission.get("id"),
            "missions": steps,
            "createdBy": mission.get("createdBy"),
        }


def main(args=None):
    rclpy.init(args=args)
    node = MissionApiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
