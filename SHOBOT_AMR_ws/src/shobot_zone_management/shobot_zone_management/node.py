#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class ZoneManagementNode(Node):
    """Placeholder zone monitor that reports zone names based on simple radius checks."""

    def __init__(self):
        super().__init__("shobot_zone_management")
        self.declare_parameter("pose_topic", "/robot_pose")
        self.declare_parameter("zone_topic", "/zone_status")
        self.declare_parameter("zones", [{"name": "zone_a", "x": 0.0, "y": 0.0, "r": 1.0}])

        pose_topic = self.get_parameter("pose_topic").value
        zone_topic = self.get_parameter("zone_topic").value
        self.zones = self.get_parameter("zones").value

        self.publisher = self.create_publisher(String, zone_topic, 10)
        self.subscription = self.create_subscription(
            PoseStamped, pose_topic, self.callback, 10
        )
        self.get_logger().info(f"Monitoring zones on pose {pose_topic} -> {zone_topic}")

    def callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        active = []
        for zone in self.zones:
            dx = x - float(zone.get("x", 0.0))
            dy = y - float(zone.get("y", 0.0))
            r = float(zone.get("r", 0.0))
            if dx * dx + dy * dy <= r * r:
                active.append(zone.get("name", "zone"))
        out = String()
        out.data = ",".join(active)
        self.publisher.publish(out)


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
