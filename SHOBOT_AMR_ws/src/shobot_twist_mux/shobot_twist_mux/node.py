#!/usr/bin/env python3
"""Priority-based Twist multiplexer with timeouts and safety stop."""
import time
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


@dataclass
class Source:
    name: str
    priority: int
    timeout: float
    last_msg: Optional[Twist] = None
    last_time: float = 0.0


class TwistMux(Node):
    """Choose the highest priority, non-expired Twist and publish to output."""

    def __init__(self):
        super().__init__("shobot_twist_mux")
        self.declare_parameter("sources", ["safety", "teleop", "nav"])
        self.declare_parameter("priorities", {"safety": 100, "teleop": 50, "nav": 10})
        self.declare_parameter("timeouts", {"safety": 0.5, "teleop": 0.5, "nav": 1.0})
        self.declare_parameter("topics", {"safety": "/cmd_vel/safety", "teleop": "/cmd_vel/teleop", "nav": "/cmd_vel/nav"})
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("safety_stop_topic", "/safety_stop")

        names = self.get_parameter("sources").value
        priorities = self.get_parameter("priorities").value
        timeouts = self.get_parameter("timeouts").value
        topics = self.get_parameter("topics").value
        self.output_topic = self.get_parameter("output_topic").value
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.safety_stop_topic = self.get_parameter("safety_stop_topic").value

        self.sources: Dict[str, Source] = {}
        now = self.get_clock().now().nanoseconds / 1e9
        for name in names:
            self.sources[name] = Source(
                name=name,
                priority=int(priorities.get(name, 0)),
                timeout=float(timeouts.get(name, 1.0)),
                last_time=now,
            )
            topic = topics.get(name, f"/cmd_vel/{name}")
            self.create_subscription(Twist, topic, lambda msg, n=name: self._cb(msg, n), 10)

        self.output_pub = self.create_publisher(Twist, self.output_topic, 10)
        self.create_subscription(Bool, self.safety_stop_topic, self.safety_cb, 10)
        self.safety_stop = False

        self.timer = self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(f"Twist mux publishing {self.output_topic} from sources {list(self.sources.keys())}")

    def _cb(self, msg: Twist, name: str):
        now = self.get_clock().now().nanoseconds / 1e9
        src = self.sources[name]
        self.sources[name] = Source(
            name=src.name,
            priority=src.priority,
            timeout=src.timeout,
            last_msg=msg,
            last_time=now,
        )

    def safety_cb(self, msg: Bool):
        self.safety_stop = msg.data

    def tick(self):
        if self.safety_stop:
            self.output_pub.publish(Twist())
            return

        now = self.get_clock().now().nanoseconds / 1e9
        chosen = None
        for src in self.sources.values():
            if src.last_msg is None:
                continue
            if now - src.last_time > src.timeout:
                continue
            if chosen is None or src.priority > chosen.priority:
                chosen = src

        if chosen and chosen.last_msg:
            self.output_pub.publish(chosen.last_msg)
        else:
            self.output_pub.publish(Twist())


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
