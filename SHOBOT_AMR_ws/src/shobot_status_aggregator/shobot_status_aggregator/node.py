#!/usr/bin/env python3
"""Aggregate status strings into a single JSON summary."""
import json
from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class StatusAggregator(Node):
    """Subscribe to multiple status topics and publish a combined summary."""

    def __init__(self):
        super().__init__("shobot_status_aggregator")
        self.declare_parameter(
            "status_topics",
            ["/mission_status", "/navigation_status", "/navigation_server/status", "/mission_handler_status"],
        )
        self.declare_parameter("output_topic", "/system_status")
        self.declare_parameter("rate_hz", 2.0)

        self.status_topics = self.get_parameter("status_topics").value
        self.output_topic = self.get_parameter("output_topic").value
        self.status: Dict[str, str] = {}

        for topic in self.status_topics:
            self.create_subscription(String, topic, lambda msg, t=topic: self._cb(t, msg), 10)

        self.pub = self.create_publisher(String, self.output_topic, 10)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self.publish_status)
        self.get_logger().info(f"Aggregating status from {self.status_topics} -> {self.output_topic}")

    def _cb(self, topic: str, msg: String):
        self.status[topic] = msg.data

    def publish_status(self):
        payload = {"statuses": self.status, "count": len(self.status)}
        self.pub.publish(String(data=json.dumps(payload)))


def main(args=None):
    rclpy.init(args=args)
    node = StatusAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
