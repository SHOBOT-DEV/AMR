#!/usr/bin/env python3
"""
Priority-based Twist Multiplexer with timeouts and safety stop.

- Selects the highest-priority active cmd_vel source
- Enforces per-source timeout
- Hard safety stop override
- Publishes zero velocity on failure or safety stop
"""

from dataclasses import dataclass
from functools import partial
import json
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


@dataclass
class Source:
    """Represents one cmd_vel input source."""
    name: str
    priority: int
    timeout: float
    last_msg: Optional[Twist] = None
    last_time: float = 0.0


class TwistMux(Node):
    """Selects the highest-priority non-expired Twist command."""

    def __init__(self):
        super().__init__("shobot_twist_mux")

        # ---------------- Parameters ----------------
        self.declare_parameter("sources", ["safety", "teleop", "nav"])

        self.declare_parameter(
            "priorities_json",
            '{"safety": 100, "teleop": 50, "nav": 10}',
        )
        self.declare_parameter(
            "timeouts_json",
            '{"safety": 0.5, "teleop": 0.5, "nav": 1.0}',
        )
        self.declare_parameter(
            "topics_json",
            '{"safety": "/cmd_vel/safety", '
            '"teleop": "/cmd_vel/teleop", '
            '"nav": "/cmd_vel/nav"}',
        )

        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("safety_stop_topic", "/safety_stop")

        # ---------------- Load Parameters ----------------
        names = self.get_parameter("sources").value
        priorities = self._load_map("priorities_json")
        timeouts = self._load_map("timeouts_json")
        topics = self._load_map("topics_json")

        self.output_topic = self.get_parameter("output_topic").value
        self.safety_stop_topic = self.get_parameter("safety_stop_topic").value
        rate_hz = float(self.get_parameter("rate_hz").value)

        # ---------------- Source Setup ----------------
        now = self._now()
        self.sources: Dict[str, Source] = {}

        for name in names:
            self.sources[name] = Source(
                name=name,
                priority=int(priorities.get(name, 0)),
                timeout=float(timeouts.get(name, 1.0)),
                last_time=now,
            )

            topic = topics.get(name, f"/cmd_vel/{name}")

            # Use partial() to avoid lambda capture bugs
            self.create_subscription(
                Twist,
                topic,
                partial(self._cmd_cb, name=name),
                10,
            )

        # ---------------- Publishers / Subscribers ----------------
        self.output_pub = self.create_publisher(Twist, self.output_topic, 10)
        self.create_subscription(
            Bool,
            self.safety_stop_topic,
            self.safety_cb,
            10,
        )

        self.safety_stop = False
        self._prev_safety = False

        # ---------------- Timer ----------------
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)

        self.get_logger().info(
            f"Twist Mux started\n"
            f"Output topic : {self.output_topic}\n"
            f"Sources      : {list(self.sources.keys())}"
        )

    # -------------------------------------------------
    def _now(self) -> float:
        """Return current ROS time in seconds."""
        return self.get_clock().now().seconds_nanoseconds()[0]

    # -------------------------------------------------
    def _load_map(self, param_name: str) -> Dict[str, object]:
        """Load a JSON dictionary from a string parameter."""
        raw = self.get_parameter(param_name).value
        if isinstance(raw, str):
            try:
                parsed = json.loads(raw)
                if isinstance(parsed, dict):
                    return parsed
            except json.JSONDecodeError:
                self.get_logger().warn(
                    f"Parameter '{param_name}' is invalid JSON. Using defaults."
                )
        return {}

    # -------------------------------------------------
    def _cmd_cb(self, msg: Twist, name: str):
        """Callback for cmd_vel sources."""
        src = self.sources[name]
        src.last_msg = msg
        src.last_time = self._now()

    # -------------------------------------------------
    def safety_cb(self, msg: Bool):
        """Hard safety stop callback."""
        self.safety_stop = msg.data

        if self.safety_stop and not self._prev_safety:
            self.get_logger().warn(
                "SAFETY STOP ACTIVATED → cmd_vel forced to ZERO"
            )

        self._prev_safety = self.safety_stop

    # -------------------------------------------------
    def tick(self):
        """Select and publish the highest-priority valid cmd_vel."""

        # Hard override
        if self.safety_stop:
            self.output_pub.publish(Twist())
            return

        now = self._now()
        chosen: Optional[Source] = None

        for src in self.sources.values():
            if src.last_msg is None:
                continue

            # Timeout check
            if (now - src.last_time) > src.timeout:
                continue

            # Priority arbitration
            if chosen is None or src.priority > chosen.priority:
                chosen = src

        # Publish selected command or zero
        if chosen and chosen.last_msg:
            self.output_pub.publish(chosen.last_msg)
        else:
            self.output_pub.publish(Twist())

    # -------------------------------------------------


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
