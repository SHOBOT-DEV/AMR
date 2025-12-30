#!/usr/bin/env python3
"""
Mission handler for SHOBOT AMR.

- Accepts mission commands as JSON
- Normalizes them into a mission queue format
- Publishes to mission_control / mission_queue
- Provides a service to clear all missions
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class MissionHandler(Node):
    """Parse mission commands and publish standardized mission queues."""

    def __init__(self):
        super().__init__("shobot_mission_handler")

        # ---------------- Parameters ----------------
        self.declare_parameter("command_topic", "/mission_command")
        self.declare_parameter("queue_topic", "/mission_queue")
        self.declare_parameter("status_topic", "/mission_handler_status")

        cmd_topic = self.get_parameter("command_topic").value
        self.queue_topic = self.get_parameter("queue_topic").value
        status_topic = self.get_parameter("status_topic").value

        # ---------------- Publishers / Subscribers ----------------
        self.queue_pub = self.create_publisher(String, self.queue_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.create_subscription(String, cmd_topic, self.command_cb, 10)

        # ---------------- Services ----------------
        self.create_service(Trigger, "clear_missions", self.clear_cb)

        self.get_logger().info(
            f"[MissionHandler] Listening on '{cmd_topic}' → publishing to '{self.queue_topic}'"
        )

    # ------------------------------------------------------------------
    def command_cb(self, msg: String):
        """Process incoming mission command JSON."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.publish_status(f"Invalid mission JSON: {exc}")
            return

        # Accept formats:
        # 1) { "missions": [ {...}, {...} ] }
        # 2) { "pose": {...}, "dock": false }  (single mission shorthand)
        missions = data.get("missions")

        if missions is None:
            pose = data.get("pose")
            if pose is None:
                self.publish_status("Missing 'missions' list or 'pose' object.")
                return

            missions = [
                {
                    "pose": pose,
                    "dock": bool(data.get("dock", False)),
                }
            ]

        if not isinstance(missions, list):
            self.publish_status("'missions' must be a list.")
            return

        if not missions:
            self.publish_status("Received empty mission list.")
            return

        # ---------------- Validate missions ----------------
        if not self._validate_missions(missions):
            return

        # ---------------- Build payload ----------------
        payload = {"missions": missions}

        if bool(data.get("clear", False)):
            payload["clear"] = True

        # ---------------- Publish ----------------
        self.queue_pub.publish(String(data=json.dumps(payload)))
        self.publish_status(f"Forwarded {len(missions)} mission(s) to mission queue.")

    # ------------------------------------------------------------------
    def clear_cb(self, request, response):
        """Clear mission queue via service."""
        payload = {"missions": [], "clear": True}
        self.queue_pub.publish(String(data=json.dumps(payload)))
        self.publish_status("Mission queue cleared via service.")

        response.success = True
        response.message = "Mission queue cleared"
        return response

    # ------------------------------------------------------------------
    def _validate_missions(self, missions) -> bool:
        """Validate mission structure."""
        for idx, mission in enumerate(missions):
            if not isinstance(mission, dict):
                self.publish_status(f"Mission #{idx} is not a dictionary.")
                return False

            if "pose" not in mission:
                self.publish_status(f"Mission #{idx} missing required 'pose'.")
                return False

            pose = mission["pose"]
            if not isinstance(pose, dict):
                self.publish_status(f"Mission #{idx} 'pose' must be an object.")
                return False

            for key in ("x", "y"):
                if key not in pose:
                    self.publish_status(
                        f"Mission #{idx} pose missing required field '{key}'."
                    )
                    return False

            if "dock" in mission and not isinstance(mission["dock"], bool):
                self.publish_status(
                    f"Mission #{idx} 'dock' must be boolean."
                )
                return False

        return True

    # ------------------------------------------------------------------
    def publish_status(self, text: str):
        """Publish status updates to UI and logs."""
        self.status_pub.publish(String(data=text))
        self.get_logger().info(f"[MissionHandler] {text}")


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MissionHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
