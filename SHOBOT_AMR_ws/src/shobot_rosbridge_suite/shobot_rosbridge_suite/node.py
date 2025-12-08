#!/usr/bin/env python3
"""Lightweight rosbridge monitor/announcer."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RosbridgeAnnouncer(Node):
    """Publishes rosbridge endpoint info for clients; optional heartbeat."""

    def __init__(self):
        super().__init__("shobot_rosbridge_suite")
        self.declare_parameter("host", "localhost")
        self.declare_parameter("port", 9090)
        self.declare_parameter("info_topic", "/rosbridge/info")
        self.declare_parameter("rate_hz", 1.0)

        host = self.get_parameter("host").value
        port = int(self.get_parameter("port").value)
        self.info_topic = self.get_parameter("info_topic").value
        rate = float(self.get_parameter("rate_hz").value)

        self.pub = self.create_publisher(String, self.info_topic, 10)
        self.endpoint = {"host": host, "port": port, "ws_url": f"ws://{host}:{port}"}
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f"Announcing rosbridge endpoint ws://{host}:{port} on {self.info_topic}")

    def tick(self):
        self.pub.publish(String(data=str(self.endpoint)))


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
