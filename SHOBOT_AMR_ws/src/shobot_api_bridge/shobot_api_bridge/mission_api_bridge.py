#!/usr/bin/env python3
"""
Mission API Bridge for SHOBOT AMR
---------------------------------
- Polls external REST API for mission definitions
- Converts them into ROS mission format
- Publishes on /mission_queue
- Updates remote mission status after dispatch
"""

import json
import os
from typing import Dict, List, Optional, Set

import rclpy
import requests
from rclpy.node import Node
from std_msgs.msg import String


class MissionApiBridge(Node):
    """Pull missions from HTTP API and push into ROS mission queue."""

    def __init__(self):
        super().__init__("mission_api_bridge")

        # ---------------------- Parameters ----------------------
        self.declare_parameter("mission_queue_topic", "/mission_queue")
        self.declare_parameter("poll_interval", 2.0)
        self.declare_parameter("api_url", os.environ.get("API_URL", "http://localhost:5000/api/v1"))
        self.declare_parameter("api_token", os.environ.get("API_TOKEN", ""))

        mission_queue_topic = self.get_parameter("mission_queue_topic").value
        poll_interval = float(self.get_parameter("poll_interval").value)
        self.api_url = str(self.get_parameter("api_url").value).rstrip("/")
        self.api_token = str(self.get_parameter("api_token").value)

        # ROS Publisher
        self.queue_pub = self.create_publisher(String, mission_queue_topic, 10)

        # Track dispatched missions
        self.dispatched: Set[str] = set()

        # Start periodic polling
        self.timer = self.create_timer(poll_interval, self._poll_and_dispatch)

        self.get_logger().info(
            f"Mission API Bridge running:\n"
            f"  Poll:        {poll_interval}s\n"
            f"  Queue Topic: {mission_queue_topic}\n"
            f"  API:         {self.api_url}\n"
            f"  Auth:        {'Yes' if self.api_token else 'No'}"
        )

    # ==========================================================
    #                    API Helpers
    # ==========================================================

    def _headers(self) -> Dict[str, str]:
        headers = {"Accept": "application/json", "Content-Type": "application/json"}
        if self.api_token:
            headers["Authorization"] = f"Bearer {self.api_token}"
        return headers

    def _get_missions(self) -> List[Dict]:
        """Fetch missions from the server."""
        try:
            res = requests.get(f"{self.api_url}/missions", headers=self._headers(), timeout=4)
            res.raise_for_status()
            payload = res.json()
            return payload.get("items", [])
        except Exception as exc:
            self.get_logger().warn(f"Mission fetch failed: {exc}")
            return []

    def _update_status(self, mission_id: str, status: str, notes: Optional[str] = None):
        """Mark mission in API as acknowledged/dispatched."""
        body = {"status": status}
        if notes:
            body["notes"] = notes

        try:
            requests.put(
                f"{self.api_url}/missions/{mission_id}",
                headers=self._headers(),
                data=json.dumps(body),
                timeout=4,
            )
        except Exception as exc:
            self.get_logger().warn(f"Status update failed for {mission_id}: {exc}")

    # ==========================================================
    #                Mission Polling & Dispatch
    # ==========================================================

    def _poll_and_dispatch(self):
        missions = self._get_missions()

        for mission in missions:
            # Extract mission ID safely
            mission_id = mission.get("id") or mission.get("_id")
            if not mission_id:
                continue
            if mission_id in self.dispatched:
                continue

            status = (mission.get("status") or "").lower()
            if status not in {"running", "queued", "active"}:
                continue

            payload = self._mission_to_queue_payload(mission)

            if not payload["missions"]:
                self.get_logger().warn(
                    f"Mission {mission_id} skipped: no valid waypoints."
                )
                continue

            self.queue_pub.publish(String(data=json.dumps(payload)))
            self.dispatched.add(mission_id)

            self.get_logger().info(
                f"Dispatched mission {mission_id} → queued {len(payload['missions'])} steps."
            )

            # Update API status so mission is not picked again
            self._update_status(mission_id, "Queued", notes="Mission dispatched to ROS")

    # ==========================================================
    #          Mission Payload Conversion (API → ROS)
    # ==========================================================

    def _mission_to_queue_payload(self, mission: Dict) -> Dict[str, object]:
        steps: List[Dict[str, object]] = []

        for wp in mission.get("waypoints", []):
            coord = wp.get("coordinate") or {}
            pos = coord.get("position", {})
            orient = coord.get("orientation", {})

            if "x" not in pos or "y" not in pos:
                continue

            pose = {
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
            }

            steps.append({"pose": pose, "dock": bool(wp.get("dock", False))})

        return {
            "mission_id": mission.get("id"),
            "missions": steps,
            "createdBy": mission.get("createdBy"),
        }


# ==========================================================
#                    Main Entry Point
# ==========================================================

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
