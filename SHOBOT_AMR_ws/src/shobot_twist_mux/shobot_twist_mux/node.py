#!/usr/bin/env python3
"""
Priority-based Twist multiplexer with timeouts and safety stop.
"""
from dataclasses import dataclass
from functools import partial
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


@dataclass
class Source:
    name: str
    priority: int
    timeout: float
    last_msg: Optional[Twist] = None
    last_time: float = 0.0


class TwistMux(Node):
    """Selects highest priority Twist command that is not expired."""

    def __init__(self):
        super().__init__("shobot_twist_mux")

        # ---------------- Parameters ----------------
        self.declare_parameter("sources", "")
        self.declare_parameter("priorities", "")
        self.declare_parameter("timeouts", "")
        self.declare_parameter("topics", "")

        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("safety_stop_topic", "/safety_stop")

        names = self._parse_list(self.get_parameter("sources").value, ["safety", "teleop", "nav"])
        priorities = self._parse_dict(self.get_parameter("priorities").value, {"safety": 100, "teleop": 50, "nav": 10})
        timeouts = self._parse_dict(self.get_parameter("timeouts").value, {"safety": 0.5, "teleop": 0.5, "nav": 1.0})
        topics = self._parse_dict(self.get_parameter("topics").value, {
            "safety": "/cmd_vel/safety",
            "teleop": "/cmd_vel/teleop",
            "nav": "/cmd_vel/nav",
        })

        self.output_topic = self.get_parameter("output_topic").value
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.safety_stop_topic = self.get_parameter("safety_stop_topic").value

        # ---------------- Source Setup ----------------
        now = self.get_clock().now().nanoseconds / 1e9
        self.sources: Dict[str, Source] = {}

        for name in names:
            self.sources[name] = Source(
                name=name,
                priority=int(priorities.get(name, 0)),
                timeout=float(timeouts.get(name, 1.0)),
                last_time=now,
            )

            topic = topics.get(name, f"/cmd_vel/{name}")

            # FIX: partial() prevents Python lambda capturing issues
            self.create_subscription(
                Twist, topic,
                partial(self._cb, name=name),
                10
            )

        # ---------------- Publishers ----------------
        self.output_pub = self.create_publisher(Twist, self.output_topic, 10)
        self.create_subscription(Bool, self.safety_stop_topic, self.safety_cb, 10)

        self.safety_stop = False

        # ---------------- Timer ----------------
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)

        self.get_logger().info(
            f"Twist Mux online → output={self.output_topic}, sources={list(self.sources.keys())}"
        )

    # -------------------------------------------------
    def _cb(self, msg: Twist, name: str):
        """Callback for each cmd_vel source."""
        now = self.get_clock().now().nanoseconds / 1e9
        src = self.sources[name]

        src.last_msg = msg
        src.last_time = now

    # -------------------------------------------------
    def safety_cb(self, msg: Bool):
        self.safety_stop = msg.data
        if self.safety_stop:
            self.get_logger().warn("SAFETY STOP ACTIVATED → Forcing /cmd_vel = ZERO")

    # -------------------------------------------------
    def tick(self):
        """Main loop: choose highest priority active command."""

        # Hard override
        if self.safety_stop:
            self.output_pub.publish(Twist())
            return

        now = self.get_clock().now().nanoseconds / 1e9
        chosen: Optional[Source] = None

        for src in self.sources.values():
            if src.last_msg is None:
                continue

            # Timeout handling
            if now - src.last_time > src.timeout:
                continue

            if chosen is None or src.priority > chosen.priority:
                chosen = src

        # Publish final cmd_vel
        if chosen and chosen.last_msg:
            self.output_pub.publish(chosen.last_msg)
        else:
            self.output_pub.publish(Twist())  # No valid command

    # -------------------------------------------------
    def _parse_list(self, value, default):
        if isinstance(value, list):
            return [str(v).strip() for v in value if str(v).strip()]
        if isinstance(value, str):
            text = value.strip()
            if not text:
                return default
            try:
                import json
                parsed = json.loads(text)
                if isinstance(parsed, list):
                    return [str(v).strip() for v in parsed if str(v).strip()]
            except Exception:
                pass
        return default

    def _parse_dict(self, value, default):
        if isinstance(value, dict):
            return value
        if isinstance(value, str):
            text = value.strip()
            if not text:
                return default
            try:
                import json
                parsed = json.loads(text)
                if isinstance(parsed, dict):
                    return parsed
            except Exception:
                pass
        return default


def main(args=None):
    rclpy.init(args=args)
    node = TwistMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
