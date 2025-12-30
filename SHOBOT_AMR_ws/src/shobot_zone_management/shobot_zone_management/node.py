#!/usr/bin/env python3
"""
Zone Management Node
==================================================
Monitors the robot's position and publishes the active zone(s)
based on configurable circular regions.

Author: Md Shahbaz Alam
ROS 2 Compatible
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class ZoneManagementNode(Node):
    """
    Zone monitor that reports active zone names based on radius checks.
    """

    def __init__(self):
        super().__init__("shobot_zone_management")

        # -------------------- Parameters --------------------
        self.declare_parameter("pose_topic", "/robot_pose")
        self.declare_parameter("zone_topic", "/zone_status")
        self.declare_parameter("zones_yaml", "")

        pose_topic = self.get_parameter("pose_topic").value
        zone_topic = self.get_parameter("zone_topic").value
        zones_yaml = self.get_parameter("zones_yaml").value

        self.zones = self._load_zones(zones_yaml)

        # -------------------- QoS Setup --------------------
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # -------------------- Publishers / Subscribers --------------------
        self.zone_pub = self.create_publisher(String, zone_topic, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            pose_topic,
            self.pose_callback,
            pose_qos,
        )

        # Track last published zone string to avoid repeated publishing
        self.last_zone_string = ""

        self.get_logger().info(
            "\nZone Management Node Started"
            f"\nSubscribed Pose Topic : {pose_topic}"
            f"\nPublishing Zone Topic : {zone_topic}"
            f"\nLoaded Zones          : {self.zones}"
        )

    # ----------------------------------------------------------
    # Load and validate zones
    # ----------------------------------------------------------
    def _load_zones(self, zones_yaml: str):
        """
        Load zones from YAML string and validate entries.
        """
        if not zones_yaml:
            self.get_logger().warn(
                "No zones_yaml provided. Using default zone."
            )
            zones = [{"name": "zone_a", "x": 0.0, "y": 0.0, "r": 1.0}]
        else:
            try:
                import yaml

                zones = yaml.safe_load(zones_yaml)
                if zones is None:
                    zones = []
            except Exception as exc:
                self.get_logger().error(
                    f"Failed to parse zones_yaml: {exc}"
                )
                zones = []

        valid_zones = []

        for z in zones:
            try:
                name = str(z.get("name", "zone"))
                x = float(z.get("x", 0.0))
                y = float(z.get("y", 0.0))
                r = float(z.get("r", 0.0))

                if r <= 0.0:
                    self.get_logger().warn(
                        f"Skipping zone '{name}' with non-positive radius"
                    )
                    continue

                valid_zones.append(
                    {"name": name, "x": x, "y": y, "r": r}
                )

            except Exception as exc:
                self.get_logger().error(
                    f"Invalid zone entry {z}: {exc}"
                )

        # Sort zones by name for deterministic behavior
        valid_zones.sort(key=lambda z: z["name"])

        return valid_zones

    # ----------------------------------------------------------
    # Pose Callback
    # ----------------------------------------------------------
    def pose_callback(self, msg: PoseStamped):
        """
        Called whenever a new pose is received.
        Determines which zones are active.
        """
        x = msg.pose.position.x
        y = msg.pose.position.y

        active_zones = []

        for zone in self.zones:
            dx = x - zone["x"]
            dy = y - zone["y"]
            r = zone["r"]

            # Inside circular zone check
            if (dx * dx + dy * dy) <= (r * r):
                active_zones.append(zone["name"])

        zone_string = ",".join(active_zones) if active_zones else "none"

        # Publish only if zone changed
        if zone_string != self.last_zone_string:
            self.last_zone_string = zone_string

            msg_out = String()
            msg_out.data = zone_string
            self.zone_pub.publish(msg_out)

            self.get_logger().info(
                f"Active Zones → {zone_string}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ZoneManagementNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
