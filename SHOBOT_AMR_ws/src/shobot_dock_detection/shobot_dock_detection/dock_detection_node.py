#!/usr/bin/env python3
"""
Dock Detection Aggregator
------------------------
Fuses multiple upstream dock detectors (AprilTag, ArUco, QR, etc.)
that publish geometry_msgs/PoseStamped into a unified output:

- /dock_pose      (PoseStamped)
- /dock_detected  (Bool)
- /dock_detection_status (String, JSON)

Selection policy:
- First valid (non-expired) source in configured priority order.
"""

import json
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String


@dataclass
class Detection:
    name: str
    topic: str
    timeout_sec: float
    pose: Optional[PoseStamped] = None


class DockDetectionNode(Node):
    """Fuse multiple dock detections into a single pose + detected flag."""

    def __init__(self):
        super().__init__("shobot_dock_detection")

        # ---------------- Parameters ----------------
        self.declare_parameter("sources", [])
        self.declare_parameter("dock_pose_topic", "/dock_pose")
        self.declare_parameter("dock_detected_topic", "/dock_detected")
        self.declare_parameter("pose_frame_override", "")
        self.declare_parameter("publish_status", True)
        self.declare_parameter("publish_status_topic", "/dock_detection_status")

        self.dock_pose_topic = self.get_parameter("dock_pose_topic").value
        self.dock_detected_topic = self.get_parameter("dock_detected_topic").value
        self.frame_override = self.get_parameter("pose_frame_override").value
        self.publish_status = bool(self.get_parameter("publish_status").value)
        self.status_topic = self.get_parameter("publish_status_topic").value

        # ---------------- Sources ----------------
        self.sources: List[Detection] = self._parse_sources(
            self.get_parameter("sources").value
        )

        if not self.sources:
            self.get_logger().warn("No dock detection sources configured!")

        # ---------------- QoS ----------------
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        for src in self.sources:
            self.create_subscription(
                PoseStamped,
                src.topic,
                self._make_cb(src),
                qos,
            )

        # ---------------- Publishers ----------------
        self.pose_pub = self.create_publisher(
            PoseStamped, self.dock_pose_topic, 10
        )
        self.detected_pub = self.create_publisher(
            Bool, self.dock_detected_topic, 10
        )
        self.status_pub = self.create_publisher(
            String, self.status_topic, 10
        )

        # ---------------- Timer ----------------
        self.create_timer(0.1, self._publish_best)  # 10 Hz

        self.get_logger().info(
            f"DockDetection started with sources: "
            f"{[(s.name, s.topic, s.timeout_sec) for s in self.sources]}"
        )

    # ------------------------------------------------------------------
    def _parse_sources(self, entries) -> List[Detection]:
        """Parse source definitions: name:topic:timeout_sec."""
        sources: List[Detection] = []

        if not entries:
            return sources

        if isinstance(entries, str):
            entries = [entries]

        for raw in entries:
            parts = str(raw).strip().split(":")
            if len(parts) != 3:
                self.get_logger().warn(
                    f"Invalid source '{raw}'. Expected 'name:topic:timeout_sec'."
                )
                continue

            name, topic, timeout_str = parts
            try:
                timeout = float(timeout_str)
            except ValueError:
                self.get_logger().warn(
                    f"Invalid timeout '{timeout_str}' for source '{raw}'."
                )
                continue

            sources.append(
                Detection(name=name, topic=topic, timeout_sec=timeout)
            )

        return sources

    # ------------------------------------------------------------------
    def _make_cb(self, src: Detection):
        """Create subscription callback bound to a specific source."""

        def _cb(msg: PoseStamped):
            # Copy message to avoid mutating ROS-managed memory
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = msg.pose

            if self.frame_override:
                pose.header.frame_id = self.frame_override

            src.pose = pose

        return _cb

    # ------------------------------------------------------------------
    def _publish_best(self):
        now = self.get_clock().now()
        best = self._select_best(now)

        detected = best is not None
        self.detected_pub.publish(Bool(data=detected))

        if detected and best.pose:
            self.pose_pub.publish(best.pose)

        if self.publish_status:
            self._publish_status(now, best, detected)

    # ------------------------------------------------------------------
    def _select_best(self, now: Time) -> Optional[Detection]:
        """Select first non-expired detection by priority order."""
        for src in self.sources:
            if src.pose is None:
                continue

            stamp = Time.from_msg(src.pose.header.stamp)
            if stamp.nanoseconds == 0:
                continue

            age = (now - stamp).nanoseconds * 1e-9
            if age <= src.timeout_sec:
                return src

        return None

    # ------------------------------------------------------------------
    def _publish_status(
        self, now: Time, best: Optional[Detection], detected: bool
    ):
        """Publish JSON status for diagnostics."""
        status: Dict[str, Dict] = {}

        for src in self.sources:
            age = None
            alive = False

            if src.pose:
                stamp = Time.from_msg(src.pose.header.stamp)
                if stamp.nanoseconds != 0:
                    age = (now - stamp).nanoseconds * 1e-9
                    alive = age <= src.timeout_sec

            status[src.name] = {
                "topic": src.topic,
                "age_sec": age,
                "timeout_sec": src.timeout_sec,
                "alive": alive,
            }

        payload = {
            "timestamp_ns": now.nanoseconds,
            "detected": detected,
            "selected_source": best.name if best else None,
            "frame_id": best.pose.header.frame_id if best else None,
            "sources": status,
        }

        self.status_pub.publish(String(data=json.dumps(payload)))


# ------------------------------------------------------------------
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
