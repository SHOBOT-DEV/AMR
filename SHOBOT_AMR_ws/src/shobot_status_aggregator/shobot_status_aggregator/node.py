#!/usr/bin/env python3
"""
Status Aggregator Node
=========================================================
Collects multiple status topics and publishes a combined JSON summary.
"""

import json
from functools import partial
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String


class StatusAggregator(Node):
    """Aggregates multiple status topics into a single JSON status."""

    def __init__(self):
        super().__init__("shobot_status_aggregator")

        # ---------------- Parameters ----------------
        self.declare_parameter(
            "status_topics",
            [
                "/mission_status",
                "/navigation_status",
                "/navigation_server/status",
                "/mission_handler_status",
            ],
        )
        self.declare_parameter("output_topic", "/system_status")
        self.declare_parameter("rate_hz", 2.0)

        self.status_topics = self.get_parameter("status_topics").value
        self.output_topic = self.get_parameter("output_topic").value
        self.rate_hz = max(float(self.get_parameter("rate_hz").value), 0.1)

        # ---------------- QoS ----------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---------------- State ----------------
        # Initialize all topics as "unknown"
        self.status: Dict[str, str] = {
            topic: "unknown" for topic in self.status_topics
        }

        # ---------------- Subscriptions ----------------
        for topic in self.status_topics:
            self.create_subscription(
                String,
                topic,
                partial(self._cb, topic=topic),
                qos,
            )

        # ---------------- Publisher ----------------
        self.pub = self.create_publisher(String, self.output_topic, 10)

        # ---------------- Timer ----------------
        self.create_timer(1.0 / self.rate_hz, self.publish_status)

        self.get_logger().info(
            f"StatusAggregator started\n"
            f"Subscribed topics: {self.status_topics}\n"
            f"Publishing to: {self.output_topic} @ {self.rate_hz} Hz"
        )

    # ------------------------------------------------------------------
    def _cb(self, msg: String, topic: str):
        """Update the stored status for a topic."""
        self.status[topic] = msg.data

    # ------------------------------------------------------------------
    def publish_status(self):
        """Publish combined JSON summary."""
        payload = {
            "timestamp_ns": self.get_clock().now().nanoseconds,
            "count": len(self.status),
            "statuses": self.status,
        }

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
