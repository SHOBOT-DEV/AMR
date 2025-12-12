#!/usr/bin/env python3
"""
Rosbridge endpoint announcer for SHOBOT AMR
---------------------------------------------------
Publishes connection info as proper JSON and optional heartbeat.
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class RosbridgeAnnouncer(Node):
    """Announces rosbridge WebSocket URL and connection metadata."""

    def __init__(self):
        super().__init__("shobot_rosbridge_announcer")

        # ---------------- Parameters ----------------
        self.declare_parameter("host", "localhost")
        self.declare_parameter("port", 9090)
        self.declare_parameter("info_topic", "/rosbridge/info")
        self.declare_parameter("rate_hz", 1.0)
        self.declare_parameter("heartbeat", True)

        host = self.get_parameter("host").value
        port = int(self.get_parameter("port").value)
        self.info_topic = self.get_parameter("info_topic").value
        rate = float(self.get_parameter("rate_hz").value)
        heartbeat = bool(self.get_parameter("heartbeat").value)

        # ---------------- QoS for web-UI friendly behavior ----------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher = self.create_publisher(String, self.info_topic, qos)

        # Endpoint data template
        self.endpoint = {
            "host": host,
            "port": port,
            "ws_url": f"ws://{host}:{port}",
            "status": "online",
            "node": self.get_name(),
        }

        # Heartbeat timer
        if heartbeat:
            self.create_timer(1.0 / rate, self.publish_status)
            self.get_logger().info(
                f"Rosbridge heartbeat enabled → announcing ws://{host}:{port} on {self.info_topic}"
            )
        else:
            # Publish once on startup
            self.publish_status()
            self.get_logger().info(
                f"Rosbridge announcer publishing once → ws://{host}:{port}"
            )

    # ----------------------------------------------------------------------
    def publish_status(self):
        """Publish JSON info including timestamp."""
        msg = {
            **self.endpoint,
            "timestamp": self.get_clock().now().to_msg().sec,  # UNIX seconds
        }

        self.publisher.publish(String(data=json.dumps(msg)))


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = RosbridgeAnnouncer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
