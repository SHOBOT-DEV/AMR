#!/usr/bin/env python3
"""
Log Recorder + Playback node.

Records configured ROS topics into newline-delimited JSON files with serialized
message payloads. Provides a playback service to re-publish recorded messages at
the original cadence (or scaled by playback_rate).
"""

import base64
import importlib
import json
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.serialization import serialize_message, deserialize_message
from std_srvs.srv import Trigger

DEFAULT_TOPIC_TYPES = {
    "mission_topic": "std_msgs.msg.String",
    "task_failure_topic": "std_msgs.msg.String",
    "error_topic": "std_msgs.msg.String",
}


@dataclass
class TopicConfig:
    topic: str
    type_str: Optional[str] = None  # e.g. "std_msgs.msg.String"


class LogRecorderNode(Node):
    """Record ROS messages to disk and provide a playback service."""

    def __init__(self):
        super().__init__("shobot_log_recorder")

        # Parameters
        self.declare_parameter("output_dir", "~/.shobot/log_recorder")
        self.declare_parameter("max_file_size_mb", 200.0)
        self.declare_parameter("playback_rate", 1.0)
        self.declare_parameter("sensor_topics", [])
        self.declare_parameter("mission_topic", "/mission_status")
        self.declare_parameter("task_failure_topic", "/task_failures")
        self.declare_parameter("error_topic", "/system/errors")
        self.declare_parameter("playback_file", "")

        self.output_dir = Path(self.get_parameter("output_dir").value).expanduser()
        self.max_file_size_bytes = float(self.get_parameter("max_file_size_mb").value) * 1e6
        self.playback_rate = float(self.get_parameter("playback_rate").value)

        # Build topic list
        self.topic_configs = self._gather_topics()

        # File rotation state
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.current_file = self._open_new_file()
        self.current_size = self.current_file.stat().st_size
        self.write_lock = threading.Lock()

        # Playback state
        self.playback_thread: Optional[threading.Thread] = None
        self.playback_stop = threading.Event()

        # Subscriptions
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        for cfg in self.topic_configs.values():
            msg_type = self._resolve_type(cfg)
            self.create_subscription(
                msg_type=msg_type,
                topic=cfg.topic,
                callback=self._make_record_cb(cfg),
                qos_profile=qos,
            )
            self.get_logger().info(f"Recording topic '{cfg.topic}' as {cfg.type_str or 'auto'}")

        # Services
        self.create_service(Trigger, "start_playback", self._handle_start_playback)
        self.create_service(Trigger, "stop_playback", self._handle_stop_playback)

        self.get_logger().info(
            f"Log recorder ready. Writing to {self.output_dir} "
            f"(max {self.max_file_size_bytes/1e6:.1f} MB/file)."
        )

    # ------------------------------------------------------------------ #
    def _gather_topics(self) -> Dict[str, TopicConfig]:
        """Collect topic configs from params."""
        topics: Dict[str, TopicConfig] = {}

        def add_topic(name: str, topic: str, type_str: Optional[str]):
            if not topic:
                return
            topics[topic] = TopicConfig(topic=topic, type_str=type_str)

        # Sensor topics: list of "topic[:type]" strings
        for entry in self._normalize_list(self.get_parameter("sensor_topics").value):
            topic, type_str = self._split_topic_entry(entry)
            add_topic(topic, topic, type_str)

        # Mission + failure + errors (default to std_msgs/String)
        for param_name in ["mission_topic", "task_failure_topic", "error_topic"]:
            topic_val = self.get_parameter(param_name).value
            type_str = DEFAULT_TOPIC_TYPES.get(param_name)
            add_topic(param_name, topic_val, type_str)

        return topics

    def _split_topic_entry(self, entry: str) -> Tuple[str, Optional[str]]:
        """Split 'topic[:type]' string."""
        if ":" not in entry:
            return entry.strip(), None
        topic, type_str = entry.split(":", 1)
        return topic.strip(), type_str.strip() or None

    def _normalize_list(self, val) -> list:
        if val is None:
            return []
        if isinstance(val, list):
            return [str(v).strip() for v in val if str(v).strip()]
        if isinstance(val, str):
            text = val.strip()
            if text.startswith("[") and text.endswith("]"):
                inner = text[1:-1]
                return [item.strip().strip("\"'") for item in inner.split(",") if item.strip().strip("\"'")]
            if text:
                return [text]
        return []

    def _resolve_type(self, cfg: TopicConfig):
        """Import message type if available; otherwise subscribe with AnyMsg."""
        if cfg.type_str:
            msg_cls = self._import_type(cfg.type_str)
            if msg_cls:
                return msg_cls
        # Try to infer from current graph
        topic_types = dict(self.get_topic_names_and_types())
        if cfg.topic in topic_types and topic_types[cfg.topic]:
            inferred = topic_types[cfg.topic][0]
            msg_cls = self._import_type(inferred)
            if msg_cls:
                cfg.type_str = inferred
                return msg_cls

        from rclpy.any_msg import AnyMsg

        cfg.type_str = None
        self.get_logger().warn(
            f"Topic '{cfg.topic}' type unknown; subscribing as AnyMsg (playback skip if not serializable)."
        )
        return AnyMsg

    def _import_type(self, type_str: str):
        """Import 'pkg.module.MsgType' safely."""
        try:
            module_name, class_name = type_str.rsplit(".", 1)
            module = importlib.import_module(module_name)
            return getattr(module, class_name)
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().warn(f"Could not import type '{type_str}': {exc}")
            return None

    # ------------------------------------------------------------------ #
    def _make_record_cb(self, cfg: TopicConfig):
        def _cb(msg):
            entry = self._build_entry(cfg, msg)
            if entry is None:
                return
            self._write_entry(entry)
        return _cb

    def _build_entry(self, cfg: TopicConfig, msg) -> Optional[dict]:
        stamp = self.get_clock().now().nanoseconds
        msg_type = cfg.type_str or f"{msg.__class__.__module__}.{msg.__class__.__name__}"
        entry = {
            "stamp": stamp,
            "topic": cfg.topic,
            "type": msg_type,
        }
        try:
            serialized = serialize_message(msg)
            entry["data_b64"] = base64.b64encode(bytes(serialized)).decode("ascii")
        except Exception as exc:
            # Fallback for non-serializable AnyMsg or unknown types
            entry["text"] = str(msg)
            self.get_logger().warn_throttle(
                5.0,
                f"Unable to serialize message on {cfg.topic}: {exc}; storing text only."
            )
        return entry

    def _open_new_file(self) -> Path:
        ts = time.strftime("%Y%m%d_%H%M%S")
        path = self.output_dir / f"log_{ts}.jsonl"
        path.touch(exist_ok=False)
        self.get_logger().info(f"Opened new log file: {path}")
        return path

    def _write_entry(self, entry: dict):
        data = json.dumps(entry, separators=(",", ":")) + "\n"
        encoded = data.encode("utf-8")
        with self.write_lock:
            self._rotate_if_needed(len(encoded))
            with self.current_file.open("ab") as f:
                f.write(encoded)
            self.current_size += len(encoded)

    def _rotate_if_needed(self, incoming_bytes: int):
        if self.current_size + incoming_bytes <= self.max_file_size_bytes:
            return
        self.current_file = self._open_new_file()
        self.current_size = 0

    # ------------------------------------------------------------------ #
    def _handle_start_playback(self, _req, resp):
        path = self._resolve_playback_file()
        if path is None:
            resp.success = False
            resp.message = "No log file found."
            return resp

        if self.playback_thread and self.playback_thread.is_alive():
            resp.success = False
            resp.message = "Playback already running."
            return resp

        self.playback_stop.clear()
        self.playback_thread = threading.Thread(target=self._playback, args=(path,), daemon=True)
        self.playback_thread.start()
        resp.success = True
        resp.message = f"Started playback from {path} (rate={self.playback_rate}x)."
        return resp

    def _handle_stop_playback(self, _req, resp):
        if self.playback_thread and self.playback_thread.is_alive():
            self.playback_stop.set()
            self.playback_thread.join(timeout=1.0)
            resp.success = True
            resp.message = "Playback stopped."
        else:
            resp.success = False
            resp.message = "Playback not running."
        return resp

    def _resolve_playback_file(self) -> Optional[Path]:
        explicit = self.get_parameter("playback_file").value
        if explicit:
            path = Path(explicit).expanduser()
            if path.exists():
                return path
            self.get_logger().warn(f"Playback file '{path}' not found.")
            return None

        candidates = sorted(self.output_dir.glob("log_*.jsonl"), key=lambda p: p.stat().st_mtime, reverse=True)
        return candidates[0] if candidates else None

    def _playback(self, path: Path):
        self.get_logger().info(f"Playback: loading {path}")
        publishers: Dict[str, tuple] = {}  # topic -> (type_str, publisher, msg_cls)

        try:
            with path.open("r", encoding="utf-8") as f:
                first_stamp = None
                start_wall = time.time()
                for line in f:
                    if self.playback_stop.is_set():
                        self.get_logger().info("Playback cancelled.")
                        return
                    try:
                        entry = json.loads(line)
                    except json.JSONDecodeError:
                        self.get_logger().warn(f"Skipping malformed log line: {line[:80]}")
                        continue

                    if "data_b64" not in entry or "type" not in entry:
                        continue

                    msg_cls = self._import_type(entry["type"])
                    if msg_cls is None:
                        continue
                    topic = entry["topic"]
                    pub = self._get_publisher(publishers, topic, msg_cls, entry["type"])
                    if pub is None:
                        continue

                    # Timing alignment
                    if first_stamp is None:
                        first_stamp = entry["stamp"]
                    recorded_dt = (entry["stamp"] - first_stamp) / 1e9
                    target = start_wall + recorded_dt / max(self.playback_rate, 0.01)
                    now = time.time()
                    if target > now:
                        time.sleep(target - now)

                    try:
                        raw = base64.b64decode(entry["data_b64"])
                        msg = deserialize_message(raw, msg_cls)
                        pub.publish(msg)
                    except Exception as exc:
                        self.get_logger().warn(f"Playback decode failed for {topic}: {exc}")
        finally:
            self.get_logger().info("Playback finished.")
            self.playback_stop.clear()

    def _get_publisher(self, cache: Dict[str, tuple], topic: str, msg_cls, type_str: str):
        if topic in cache:
            return cache[topic][1]
        try:
            pub = self.create_publisher(msg_cls, topic, 10)
            cache[topic] = (type_str, pub, msg_cls)
            return pub
        except Exception as exc:
            self.get_logger().warn(f"Could not create publisher for {topic}: {exc}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = LogRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
