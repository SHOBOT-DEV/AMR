#!/usr/bin/env python3
"""Localization monitor: checks incoming localization inputs and reports health."""
import time
from typing import Dict, List

import rclpy
from rclpy.any_msg import AnyMsg
from rclpy.node import Node
from std_msgs.msg import Bool, String


class LocalizationMonitor(Node):
    """Monitor required localization inputs and publish a simple health status."""

    def __init__(self):
        super().__init__("shobot_localization_monitor")
        self.declare_parameter(
            "input_topics",
            ["/odom", "/odom/wheel", "/rtabmap/odom", "/lidar_odom", "/vo", "/imu/data"],
        )
        self.declare_parameter("stale_timeout_sec", 1.0)
        self.declare_parameter("status_topic", "/localization/status")
        self.declare_parameter("healthy_topic", "/localization/healthy")
        self.declare_parameter("check_period_sec", 0.5)

        topics: List[str] = [
            str(t) for t in self.get_parameter("input_topics").value if str(t).strip()
        ]
        self.stale_timeout = float(self.get_parameter("stale_timeout_sec").value)
        status_topic = self.get_parameter("status_topic").value
        healthy_topic = self.get_parameter("healthy_topic").value
        check_period = float(self.get_parameter("check_period_sec").value)

        self.last_seen: Dict[str, float] = {t: 0.0 for t in topics}
        self.subs = []
        for topic in topics:
            # Use AnyMsg to accept any type without declaring message types here.
            self.subs.append(self.create_subscription(AnyMsg, topic, self._make_cb(topic), 10))

        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.healthy_pub = self.create_publisher(Bool, healthy_topic, 10)

        self.create_timer(check_period, self.check_inputs)
        self.get_logger().info(f"Monitoring localization inputs: {topics}")

    def _make_cb(self, topic: str):
        def cb(_msg):
            self.last_seen[topic] = time.time()

        return cb

    def check_inputs(self):
        now = time.time()
        stale = [t for t, ts in self.last_seen.items() if now - ts > self.stale_timeout]
        healthy = len(stale) == 0
        if stale:
            msg = f"Localization inputs stale: {stale}"
        else:
            msg = "Localization inputs healthy"
        self.status_pub.publish(String(data=msg))
        self.healthy_pub.publish(Bool(data=healthy))
        if stale:
            self.get_logger().warn(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
