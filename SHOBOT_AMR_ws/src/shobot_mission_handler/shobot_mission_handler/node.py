#!/usr/bin/env python3
"""Mission handler: normalize mission commands and forward to mission_control."""
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class MissionHandler(Node):
    """Parse mission commands and publish standardized mission queues."""

    def __init__(self):
        super().__init__("shobot_mission_handler")
        self.declare_parameter("command_topic", "/mission_command")
        self.declare_parameter("queue_topic", "/mission_queue")
        self.declare_parameter("status_topic", "/mission_handler_status")

        cmd_topic = self.get_parameter("command_topic").value
        self.queue_topic = self.get_parameter("queue_topic").value
        status_topic = self.get_parameter("status_topic").value

        self.queue_pub = self.create_publisher(String, self.queue_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.create_subscription(String, cmd_topic, self.command_cb, 10)
        self.create_service(Trigger, "clear_missions", self.clear_cb)

        self.get_logger().info(f"Mission handler listening on {cmd_topic} -> {self.queue_topic}")

    def command_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.publish_status(f"Invalid mission JSON: {e}")
            return

        missions = data.get("missions")
        if missions is None:
            # Allow single mission shorthand
            if "pose" in data:
                missions = [dict(pose=data["pose"], dock=data.get("dock", False))]
            else:
                self.publish_status("Mission command missing 'missions' or 'pose'.")
                return

        if not isinstance(missions, list):
            self.publish_status("Field 'missions' must be a list.")
            return

        payload = {"missions": missions}
        if data.get("clear"):
            payload["clear"] = True

        self.queue_pub.publish(String(data=json.dumps(payload)))
        self.publish_status(f"Forwarded {len(missions)} mission(s) to {self.queue_topic}")

    def clear_cb(self, request, response):
        payload = {"missions": [], "clear": True}
        self.queue_pub.publish(String(data=json.dumps(payload)))
        self.publish_status("Cleared missions via service call.")
        response.success = True
        response.message = "Cleared missions"
        return response

    def publish_status(self, text: str):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)


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
