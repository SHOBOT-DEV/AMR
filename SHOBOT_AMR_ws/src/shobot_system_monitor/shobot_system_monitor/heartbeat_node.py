#!/usr/bin/env python3
"""
System Heartbeat / Liveness Node for SHOBOT
==========================================

Publishes /system/heartbeat containing:
- CPU usage, memory usage, load averages
- Sensor/topic aliveness (configurable topics + timeout)
- Node/topic aliveness (configurable topics + timeout)
"""

import json
import os
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
except ModuleNotFoundError:  # optional dependency
    psutil = None


# ============================================================
# Dataclass for topic watches
# ============================================================
@dataclass
class WatchConfig:
    name: str
    topic: str
    timeout_sec: float
    type_str: str
    last_seen: Optional[Time] = None


# ============================================================
# Heartbeat Node
# ============================================================
class HeartbeatNode(Node):
    """Heartbeat publisher with system stats and topic liveness checks."""

    def __init__(self):
        super().__init__("shobot_system_heartbeat")

        # ---------------- Parameters ----------------
        self.declare_parameter("heartbeat_topic", "/system/heartbeat")
        self.declare_parameter("rate_hz", 1.0)

        # list of: "name:topic:timeout[:type]"
        self.declare_parameter("sensor_topics", [])
        self.declare_parameter("node_topics", [])

        self.heartbeat_topic = self.get_parameter("heartbeat_topic").value
        self.rate_hz = max(float(self.get_parameter("rate_hz").value), 0.1)

        self.sensor_watch = self._parse_watch_list(
            self.get_parameter("sensor_topics").value, "sensor_topics"
        )
        self.node_watch = self._parse_watch_list(
            self.get_parameter("node_topics").value, "node_topics"
        )

        # ---------------- QoS ----------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---------------- Subscriptions ----------------
        for watch in self.sensor_watch + self.node_watch:
            msg_type = self._resolve_type(watch.type_str)
            self.create_subscription(
                msg_type,
                watch.topic,
                self._make_cb(watch),
                qos,
            )

        # ---------------- Publisher ----------------
        self.pub = self.create_publisher(String, self.heartbeat_topic, 10)

        # Prime psutil CPU
        if psutil:
            psutil.cpu_percent(None)

        # ---------------- Timer ----------------
        self.create_timer(1.0 / self.rate_hz, self._publish_heartbeat)

        self.get_logger().info(
            f"Heartbeat active → topic={self.heartbeat_topic}, rate={self.rate_hz} Hz "
            f"(sensors={len(self.sensor_watch)}, nodes={len(self.node_watch)})"
        )

    # ------------------------------------------------------------
    # Parsing helpers
    # ------------------------------------------------------------
    def _parse_watch_list(self, entries, param_name: str) -> List[WatchConfig]:
        """Parse list of 'name:topic:timeout[:type]'."""
        normalized = self._normalize_entries(entries)
        result: List[WatchConfig] = []

        for raw in normalized:
            parts = str(raw).split(":")
            if len(parts) not in (3, 4):
                self.get_logger().warn(
                    f"{param_name}: '{raw}' should be name:topic:timeout[:type]"
                )
                continue

            name, topic, timeout_str = parts[:3]
            type_str = parts[3] if len(parts) == 4 else "std_msgs/msg/String"

            try:
                timeout = float(timeout_str)
            except ValueError:
                self.get_logger().warn(
                    f"{param_name}: invalid timeout '{timeout_str}' in '{raw}'"
                )
                continue

            result.append(
                WatchConfig(
                    name=name,
                    topic=topic,
                    timeout_sec=timeout,
                    type_str=type_str,
                )
            )

        return result

    def _normalize_entries(self, entries) -> List[str]:
        """Normalize parameter to list of strings."""
        if entries is None:
            return []
        if isinstance(entries, list):
            return [str(e).strip() for e in entries if str(e).strip()]
        if isinstance(entries, str) and entries.strip():
            return [entries.strip()]
        return []

    # ------------------------------------------------------------
    # ROS helpers
    # ------------------------------------------------------------
    def _resolve_type(self, type_str: str):
        try:
            return get_message(type_str)
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to load message type '{type_str}', using std_msgs/String ({exc})"
            )
            return String

    def _make_cb(self, watch: WatchConfig):
        def _cb(_msg):
            watch.last_seen = self.get_clock().now()
        return _cb

    # ------------------------------------------------------------
    # System stats
    # ------------------------------------------------------------
    def _collect_system_stats(self):
        """Collect CPU / memory / load averages."""
        try:
            load_avg = os.getloadavg()
        except OSError:
            load_avg = (None, None, None)

        if psutil:
            cpu_percent = psutil.cpu_percent(interval=None)
            mem_percent = psutil.virtual_memory().percent
        else:
            cpu_count = os.cpu_count() or 1
            cpu_percent = (
                min(100.0, (load_avg[0] / cpu_count) * 100.0)
                if load_avg[0] is not None
                else None
            )
            mem_percent = None

        return cpu_percent, mem_percent, load_avg

    # ------------------------------------------------------------
    # Watch status
    # ------------------------------------------------------------
    def _watch_status(self, watch: WatchConfig) -> Dict:
        now = self.get_clock().now()
        alive = False
        age_sec = None

        if watch.last_seen:
            age_sec = (now - watch.last_seen).nanoseconds / 1e9
            alive = age_sec <= watch.timeout_sec

        return {
            "name": watch.name,
            "topic": watch.topic,
            "alive": alive,
            "age_sec": age_sec,
            "timeout_sec": watch.timeout_sec,
        }

    # ------------------------------------------------------------
    # Heartbeat publish
    # ------------------------------------------------------------
    def _publish_heartbeat(self):
        cpu_percent, mem_percent, load_avg = self._collect_system_stats()

        sensors = [self._watch_status(w) for w in self.sensor_watch]
        nodes = [self._watch_status(w) for w in self.node_watch]

        all_alive = all(item["alive"] for item in sensors + nodes) if (sensors or nodes) else True

        payload = {
            "timestamp_ns": self.get_clock().now().nanoseconds,
            "cpu_percent": cpu_percent,
            "mem_percent": mem_percent,
            "load_avg": load_avg,
            "sensors": sensors,
            "nodes": nodes,
            "ok": all_alive,
        }

        self.pub.publish(String(data=json.dumps(payload)))

        if not all_alive:
            missing = [i["name"] for i in sensors + nodes if not i["alive"]]
            self.get_logger().warn(f"Heartbeat degraded: missing [{', '.join(missing)}]")


# ============================================================
# Main
# ============================================================
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
