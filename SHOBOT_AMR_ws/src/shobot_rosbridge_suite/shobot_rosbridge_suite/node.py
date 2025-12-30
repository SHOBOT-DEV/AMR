#!/usr/bin/env python3
"""
Rosbridge Endpoint Announcer for SHOBOT AMR
==========================================

Publishes rosbridge WebSocket connection information as JSON.
Optionally republishes periodically as a heartbeat.
"""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String


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
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.heartbeat = bool(self.get_parameter("heartbeat").value)

        # Guard against invalid rate
        self.rate_hz = max(rate_hz, 0.1)

        # ---------------- QoS ----------------
        # Transient local ensures late subscribers (web UI) receive last message
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.publisher = self.create_publisher(String, self.info_topic, qos)

        # Endpoint info template
        self.endpoint = {
            "host": host,
            "port": port,
            "ws_url": f"ws://{host}:{port}",
            "node": self.get_name(),
        }

        # ---------------- Timers ----------------
        if self.heartbeat:
            self.create_timer(1.0 / self.rate_hz, self.publish_status)
            self.get_logger().info(
                f"Rosbridge announcer started (heartbeat @ {self.rate_hz} Hz)\n"
                f"Endpoint: ws://{host}:{port}\n"
                f"Topic   : {self.info_topic}"
            )
        else:
            # Publish once
            self.publish_status()
            self.get_logger().info(
                f"Rosbridge announcer published once → ws://{host}:{port}"
            )

    # -------------------------------------------------
    def publish_status(self):
        """Publish rosbridge endpoint information as JSON."""
        payload = {
            **self.endpoint,
            "status": "online",
            "timestamp_ns": self.get_clock().now().nanoseconds,
        }

        self.publisher.publish(String(data=json.dumps(payload)))


# ==================================================
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
