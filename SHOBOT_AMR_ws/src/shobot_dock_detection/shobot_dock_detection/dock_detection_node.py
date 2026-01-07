#!/usr/bin/env python3
"""
Dock station detection aggregator.

Use upstream detectors (AprilTag, QR, ArUco, magnetic/colored markers) that output
geometry_msgs/PoseStamped. This node merges multiple sources into a unified:
- /dock_pose (PoseStamped)
- /dock_detected (Bool)
"""

import json
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import Bool, String


@dataclass
class Detection:
    name: str
    topic: str
    timeout_sec: float
    pose: Optional[PoseStamped] = None


class DockDetectionNode(Node):
    """Fuse multiple dock detections into a single pose + flag."""

    def __init__(self):
        super().__init__("shobot_dock_detection")

        # Parameters
        self.declare_parameter(
            "sources",
            "",
            descriptor=ParameterDescriptor(
                description="Comma-separated list: name:topic:timeout_sec",
            ),
        )
        self.declare_parameter("dock_pose_topic", "/dock_pose")
        self.declare_parameter("dock_detected_topic", "/dock_detected")
        self.declare_parameter("pose_frame_override", "")  # override frame_id if non-empty
        self.declare_parameter("publish_status_topic", "/dock_detection_status")
        self.declare_parameter("publish_status", True)

        self.dock_pose_topic = self.get_parameter("dock_pose_topic").value
        self.dock_detected_topic = self.get_parameter("dock_detected_topic").value
        self.frame_override = self.get_parameter("pose_frame_override").value
        self.status_topic = self.get_parameter("publish_status_topic").value
        self.publish_status = bool(self.get_parameter("publish_status").value)

        self.sources = self._parse_sources(self.get_parameter("sources").value)
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        for src in self.sources:
            self.create_subscription(
                PoseStamped,
                src.topic,
                self._make_cb(src),
                qos,
            )

        self.pose_pub = self.create_publisher(PoseStamped, self.dock_pose_topic, 10)
        self.detected_pub = self.create_publisher(Bool, self.dock_detected_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        # Timer to publish best detection
        self.create_timer(0.1, self._publish_best)  # 10 Hz lightweight loop

        self.get_logger().info(
            f"DockDetection: watching sources {[(s.name, s.topic) for s in self.sources]} "
            f"-> {self.dock_pose_topic}, {self.dock_detected_topic}"
        )

    # ------------------------------------------------------------------ #
    def _parse_sources(self, entries) -> List[Detection]:
        normalized = self._normalize_entries(entries)
        sources: List[Detection] = []
        for raw in normalized:
            parts = str(raw).split(":")
            if len(parts) != 3:
                self.get_logger().warn(
                    f"Source '{raw}' should be 'name:topic:timeout_sec'; skipping."
                )
                continue
            name, topic, timeout_str = parts
            try:
                timeout = float(timeout_str)
            except ValueError:
                self.get_logger().warn(f"Timeout '{timeout_str}' invalid for '{raw}'; skipping.")
                continue
            sources.append(Detection(name=name, topic=topic, timeout_sec=timeout))
        return sources

    def _normalize_entries(self, entries) -> List[str]:
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

    def _make_cb(self, src: Detection):
        def _cb(msg: PoseStamped):
            # Keep most recent pose and timestamp
            if self.frame_override:
                msg.header.frame_id = self.frame_override
            src.pose = msg
        return _cb

    def _publish_best(self):
        now = self.get_clock().now()
        best = self._select_best(now)

        detected = best is not None
        self.detected_pub.publish(Bool(data=detected))

        if detected:
            self.pose_pub.publish(best.pose)

        if self.publish_status:
            self._publish_status(now, best, detected)

    def _select_best(self, now: Time) -> Optional[Detection]:
        # Priority: first valid in list order
        for src in self.sources:
            if src.pose is None:
                continue
            age = (now - Time.from_msg(src.pose.header.stamp)).nanoseconds / 1e9
            if age <= src.timeout_sec:
                return src
        return None

    def _publish_status(self, now: Time, best: Optional[Detection], detected: bool):
        statuses: Dict[str, Dict] = {}
        for src in self.sources:
            age = None
            if src.pose:
                age = (now - Time.from_msg(src.pose.header.stamp)).nanoseconds / 1e9
            statuses[src.name] = {
                "topic": src.topic,
                "age_sec": age,
                "timeout_sec": src.timeout_sec,
                "alive": age is not None and age <= src.timeout_sec,
            }
        payload = {
            "timestamp": now.nanoseconds,
            "detected": detected,
            "selected_source": best.name if best else None,
            "sources": statuses,
            "frame_id": best.pose.header.frame_id if best else None,
        }
        self.status_pub.publish(String(data=json.dumps(payload)))


def main(args=None):
    rclpy.init(args=args)
    node = DockDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
