#!/usr/bin/env python3
"""
Mission handler: normalize mission commands and forward to mission_control.
Enhanced version with validation, clearer logs, and robust error handling.
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

        self.get_logger().info(f"[MissionHandler] Listening on {cmd_topic} → {self.queue_topic}")

    # ------------------------------------------------------------------
    def command_cb(self, msg: String):
        """Process incoming mission JSON."""
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.publish_status(f"❌ Invalid mission JSON: {e}")
            return

        # Accept either:
        # 1) { "missions": [ ... ] }
        # 2) { "pose": {...}, "dock": false } → converted automatically
        missions = data.get("missions")

        if missions is None:
            # Shorthand mode
            pose = data.get("pose")
            if not pose:
                self.publish_status("❌ Missing 'missions' list or 'pose' object.")
                return

            missions = [dict(pose=pose, dock=data.get("dock", False))]

        if not isinstance(missions, list):
            self.publish_status("❌ 'missions' must be a list.")
            return

        if len(missions) == 0:
            self.publish_status("⚠️ Received empty mission list.")

        # ---------------- Validate missions ----------------
        if not self._validate_missions(missions):
            return

        # ---------------- Build final payload ----------------
        payload = {"missions": missions}

        if data.get("clear"):
            payload["clear"] = True

        # ---------------- Publish to mission queue ----------------
        self.queue_pub.publish(String(data=json.dumps(payload)))
        self.publish_status(f"➡️ Forwarded {len(missions)} mission(s) to {self.queue_topic}")

    # ------------------------------------------------------------------
    def clear_cb(self, request, response):
        """Service call to clear mission queue immediately."""
        payload = {"missions": [], "clear": True}
        self.queue_pub.publish(String(data=json.dumps(payload)))
        self.publish_status("🧹 Cleared missions via service call.")
        response.success = True
        response.message = "Cleared missions"
        return response

    # ------------------------------------------------------------------
    def _validate_missions(self, missions):
        """Validate mission list and return False if invalid."""
        for idx, m in enumerate(missions):
            if "pose" not in m:
                self.publish_status(f"❌ Mission #{idx} missing required 'pose' field.")
                return False

            pose = m["pose"]
            required = ["x", "y"]
            for key in required:
                if key not in pose:
                    self.publish_status(
                        f"❌ Mission #{idx} pose missing required field '{key}'. Pose={pose}"
                    )
                    return False

            # Docking missions optional
            if "dock" in m and not isinstance(m["dock"], bool):
                self.publish_status(
                    f"❌ Mission #{idx} 'dock' must be true/false, got: {m['dock']}"
                )
                return False

        return True

    # ------------------------------------------------------------------
    def publish_status(self, text: str):
        """Send status updates to UI + log output."""
        msg = String(data=text)
        self.status_pub.publish(msg)
        self.get_logger().info(f"[MissionHandler] {text}")


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MissionHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
