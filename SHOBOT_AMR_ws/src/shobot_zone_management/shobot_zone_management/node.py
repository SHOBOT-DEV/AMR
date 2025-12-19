#!/usr/bin/env python3
"""
Zone Management Node
==================================================
Monitors the robot’s position and publishes the active zone(s)
based on configurable circular regions.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class ZoneManagementNode(Node):
    """Zone monitor that reports active zone names based on radius checks."""

    def __init__(self):
        super().__init__("shobot_zone_management")

        # -------------------- Parameters --------------------
        self.declare_parameter("pose_topic", "/robot_pose")
        self.declare_parameter("zone_topic", "/zone_status")
        # Zones are provided as a YAML/JSON string for portability across ROS param types.
        self.declare_parameter(
            "zones_yaml",
            "",
        )

        pose_topic = self.get_parameter("pose_topic").value
        zone_topic = self.get_parameter("zone_topic").value
        self.zones = self._load_zones(self.get_parameter("zones_yaml").value)

        # -------------------- QoS Setup --------------------
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # -------------------- Publishers / Subscribers --------------------
        self.zone_pub = self.create_publisher(String, zone_topic, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, pose_topic, self.pose_callback, pose_qos
        )

        # Track last published zone message to avoid repeat
        self.last_zone_string = ""

        self.get_logger().info(
            f"Zone Management Active\n"
            f"Subscribed to pose: {pose_topic}\n"
            f"Publishing zone status on: {zone_topic}\n"
            f"Loaded Zones: {self.zones}"
        )

    # ----------------------------------------------------------
    # Validate and sanitize zone definitions
    # ----------------------------------------------------------
    def _load_zones(self, zones_yaml: str):
        if not zones_yaml:
            # Default zone if none provided
            zones = [{"name": "zone_a", "x": 0.0, "y": 0.0, "r": 1.0}]
        else:
            try:
                import yaml

                zones = yaml.safe_load(zones_yaml)
                if zones is None:
                    zones = []
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Failed to parse zones_yaml: {exc}")
                zones = []
        valid = []
        for z in zones:
            try:
                name = str(z.get("name", "zone"))
                x = float(z.get("x", 0.0))
                y = float(z.get("y", 0.0))
                r = float(z.get("r", 0.0))
                valid.append({"name": name, "x": x, "y": y, "r": r})
            except Exception as e:
                self.get_logger().error(f"Invalid zone entry {z}: {e}")
        return valid

    # ----------------------------------------------------------
    # Main Pose Callback
    # ----------------------------------------------------------
    def pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y

        active_zones = []

        for zone in self.zones:
            dx = x - zone["x"]
            dy = y - zone["y"]
            r = zone["r"]

            # Inside circular zone
            if dx * dx + dy * dy <= r * r:
                active_zones.append(zone["name"])

        zone_string = ",".join(active_zones) if active_zones else "none"

        # Publish only if zone changed
        if zone_string != self.last_zone_string:
            self.last_zone_string = zone_string

            msg_out = String()
            msg_out.data = zone_string
            self.zone_pub.publish(msg_out)

            self.get_logger().info(f"Active Zones → {zone_string}")

    # ----------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = ZoneManagementNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
