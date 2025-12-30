#!/usr/bin/env python3
"""
Localization Monitor for SHOBOT AMR
=================================================
Monitors required localization input topics and
publishes health status based on message freshness.
"""

from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String


class LocalizationMonitor(Node):
    """Monitor localization inputs and report health."""

    def __init__(self):
        super().__init__("shobot_localization_monitor")

        # ---------------- Parameters ----------------
        self.declare_parameter(
            "input_topics",
            [
                "/odom",
                "/odom/wheel",
                "/rtabmap/odom",
                "/lidar_odom",
                "/vo",
                "/imu/data",
            ],
        )
        self.declare_parameter("stale_timeout_sec", 1.0)
        self.declare_parameter("check_period_sec", 0.5)
        self.declare_parameter("status_topic", "/localization/status")
        self.declare_parameter("healthy_topic", "/localization/healthy")

        self.input_topics: List[str] = list(
            self.get_parameter("input_topics").value
        )

        self.stale_timeout = max(
            float(self.get_parameter("stale_timeout_sec").value), 0.1
        )
        self.check_period = max(
            float(self.get_parameter("check_period_sec").value), 0.1
        )

        self.status_topic = self.get_parameter("status_topic").value
        self.healthy_topic = self.get_parameter("healthy_topic").value

        # ---------------- State ----------------
        self.last_seen: Dict[str, int] = {
            topic: 0 for topic in self.input_topics
        }

        # ---------------- Subscriptions ----------------
        # Message type does NOT matter; callback only tracks activity
        for topic in self.input_topics:
            self.create_subscription(
                String,  # lightweight placeholder
                topic,
                self._make_cb(topic),
                qos_profile_sensor_data,
            )

        # ---------------- Publishers ----------------
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.healthy_pub = self.create_publisher(Bool, self.healthy_topic, 10)

        # ---------------- Timer ----------------
        self.create_timer(self.check_period, self.check_inputs)

        self.get_logger().info(
            f"LocalizationMonitor started\n"
            f"  Topics        : {self.input_topics}\n"
            f"  Timeout (sec) : {self.stale_timeout}\n"
            f"  Check period  : {self.check_period}"
        )

    # ------------------------------------------------------------------
    def _make_cb(self, topic: str):
        def cb(_msg):
            self.last_seen[topic] = self.get_clock().now().nanoseconds
        return cb

    # ------------------------------------------------------------------
    def check_inputs(self):
        now_ns = self.get_clock().now().nanoseconds
        stale_topics = []

        for topic, last_ns in self.last_seen.items():
            if last_ns == 0:
                stale_topics.append(topic)
                continue

            age_sec = (now_ns - last_ns) / 1e9
            if age_sec > self.stale_timeout:
                stale_topics.append(topic)

        healthy = len(stale_topics) == 0

        status_msg = {
            "healthy": healthy,
            "stale_topics": stale_topics,
            "timeout_sec": self.stale_timeout,
            "timestamp_ns": now_ns,
        }

        self.status_pub.publish(String(data=str(status_msg)))
        self.healthy_pub.publish(Bool(data=healthy))

        if not healthy:
            self.get_logger().warn(
                f"Localization inputs stale: {stale_topics}"
            )


# ======================================================================
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
