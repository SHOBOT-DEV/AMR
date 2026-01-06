#!/usr/bin/env python3
"""
System heartbeat / liveness node for SHOBOT.

Publishes /system/heartbeat with:
- CPU usage, memory usage, load averages
- Sensor/topic aliveness (configurable heartbeat topics + timeout)
- Node/topic aliveness (configurable heartbeat topics + timeout)
"""

import json
import os
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from std_msgs.msg import String
from rosidl_runtime_py.utilities import get_message

try:
    import psutil
except ModuleNotFoundError:  # pragma: no cover - optional dependency
    psutil = None


@dataclass
class WatchConfig:
    name: str
    topic: str
    timeout_sec: float
    last_seen: Optional[Time] = None


class HeartbeatNode(Node):
    """Heartbeat publisher with CPU/memory and topic liveness checks."""

    def __init__(self):
        super().__init__("shobot_system_heartbeat")

        # Parameters
        self.declare_parameter("heartbeat_topic", "/system/heartbeat")
        self.declare_parameter("rate_hz", 1.0)
        self.declare_parameter(
            "sensor_topics",
            "",  # list of "name:topic:timeout_sec[:type]" (type optional)
        )
        self.declare_parameter(
            "node_topics",
            "",  # list of "name:topic:timeout_sec[:type]" (type optional)
        )

        self.heartbeat_topic = self.get_parameter("heartbeat_topic").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.sensor_watch = self._parse_watch_list(
            self.get_parameter("sensor_topics").value, "sensor_topics"
        )
        self.node_watch = self._parse_watch_list(
            self.get_parameter("node_topics").value, "node_topics"
        )

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Subscriptions for sensors/nodes (Any type)
        for watch in self.sensor_watch + self.node_watch:
            msg_type = self._resolve_type(getattr(watch, "type_str", "std_msgs/msg/String"))
            self.create_subscription(
                msg_type=msg_type,
                topic=watch.topic,
                callback=self._make_cb(watch),
                qos_profile=qos,
            )

        self.pub = self.create_publisher(String, self.heartbeat_topic, 10)

        if psutil:
            # Prime psutil CPU to avoid first-call 0.0
            psutil.cpu_percent(None)

        self.create_timer(1.0 / max(self.rate_hz, 0.1), self._publish_heartbeat)

        self.get_logger().info(
            f"Heartbeat publishing to {self.heartbeat_topic} at {self.rate_hz} Hz "
            f"(sensors: {len(self.sensor_watch)}, nodes: {len(self.node_watch)})"
        )

    # ------------------------------------------------------------------ #
    def _parse_watch_list(self, entries, param_name: str) -> List[WatchConfig]:
        """Parse list like ['laser:/scan:1.0', 'imu:/imu/data:1.0:std_msgs/msg/String'] or YAML string."""
        normalized = self._normalize_entries(entries)
        result: List[WatchConfig] = []
        for raw in normalized:
            parts = str(raw).split(":")
            if len(parts) not in (3, 4):
                self.get_logger().warn(
                    f"{param_name}: '{raw}' should be 'name:topic:timeout_sec[:type]'; skipping."
                )
                continue
            name, topic, timeout = parts[:3]
            type_str = parts[3] if len(parts) == 4 else "std_msgs/msg/String"
            try:
                timeout_val = float(timeout)
            except ValueError:
                self.get_logger().warn(
                    f"{param_name}: timeout '{timeout}' invalid; skipping '{raw}'."
                )
                continue
            cfg = WatchConfig(name=name, topic=topic, timeout_sec=timeout_val)
            cfg.type_str = type_str  # type: ignore[attr-defined]
            result.append(cfg)
        return result

    def _normalize_entries(self, entries) -> List[str]:
        """Convert parameter value to a list of strings."""
        if entries is None:
            return []
        if isinstance(entries, list):
            return [str(e).strip() for e in entries if str(e).strip()]
        if isinstance(entries, str):
            text = entries.strip()
            if text.startswith("[") and text.endswith("]"):
                inner = text[1:-1]
                return [item.strip().strip("\"'") for item in inner.split(",") if item.strip().strip("\"'")]
            if text:
                return [text]
        return []

    def _resolve_type(self, type_str: str):
        """Load ROS message class from a type string like std_msgs/msg/String."""
        try:
            return get_message(type_str)
        except Exception as exc:
            self.get_logger().warn(
                f"Could not load message type '{type_str}', falling back to std_msgs/msg/String ({exc})"
            )
            return String

    def _make_cb(self, watch: WatchConfig):
        def _cb(_msg):
            watch.last_seen = self.get_clock().now()
        return _cb

    def _collect_system_stats(self):
        """Collect CPU and memory stats with psutil if available."""
        load1, load5, load15 = os.getloadavg()
        cpu_percent = None
        mem_percent = None
        if psutil:
            cpu_percent = psutil.cpu_percent(interval=None)
            mem_percent = psutil.virtual_memory().percent
        else:
            # Best-effort fallback: normalize load by cpu count to estimate %
            cpu_count = os.cpu_count() or 1
            cpu_percent = min(100.0, (load1 / cpu_count) * 100.0)
        return cpu_percent, mem_percent, (load1, load5, load15)

    def _watch_status(self, watch: WatchConfig) -> Dict:
        now = self.get_clock().now()
        age = None
        alive = False
        if watch.last_seen is not None:
            age = (now - watch.last_seen).nanoseconds / 1e9
            alive = age <= watch.timeout_sec
        return {
            "name": watch.name,
            "topic": watch.topic,
            "alive": alive,
            "age_sec": age,
            "timeout_sec": watch.timeout_sec,
        }

    def _publish_heartbeat(self):
        cpu_percent, mem_percent, loads = self._collect_system_stats()

        sensors = [self._watch_status(w) for w in self.sensor_watch]
        nodes = [self._watch_status(w) for w in self.node_watch]

        all_alive = all(item["alive"] for item in sensors + nodes) if (sensors or nodes) else True

        payload = {
            "timestamp": self.get_clock().now().nanoseconds,
            "cpu_percent": cpu_percent,
            "mem_percent": mem_percent,
            "load_avg": loads,
            "sensors": sensors,
            "nodes": nodes,
            "ok": all_alive,
        }

        self.pub.publish(String(data=json.dumps(payload)))

        # Lightweight log for debugging if something is unhealthy
        if not all_alive:
            names = [item["name"] for item in sensors + nodes if not item["alive"]]
            self.get_logger().warn(f"Heartbeat: missing [{', '.join(names)}]")


def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
