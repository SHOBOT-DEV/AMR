#!/usr/bin/env python3
"""
Localization reset helper.

Use cases:
- AMCL divergence
- Robot starts at unknown pose
- Operator manually repositions the robot

Publishes geometry_msgs/PoseWithCovarianceStamped on /initialpose.
"""

import math
from typing import List

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_srvs.srv import Trigger


class LocalizationResetNode(Node):
    """Publish /initialpose from a service or from a relayed topic."""

    def __init__(self):
        super().__init__("shobot_localization_reset")

        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("reset_pose_topic", "/reset_initialpose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("default_pose_xy_yaw", [0.0, 0.0, 0.0])  # meters, radians
        self.declare_parameter("covariance_x", 0.25)  # m^2
        self.declare_parameter("covariance_y", 0.25)  # m^2
        self.declare_parameter("covariance_yaw", math.radians(10) ** 2)  # rad^2
        self.declare_parameter("service_name", "reset_localization")

        self.initialpose_topic = self.get_parameter("initialpose_topic").value
        reset_pose_topic = self.get_parameter("reset_pose_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.default_pose = self._parse_pose(self.get_parameter("default_pose_xy_yaw").value)
        self.cov_x = float(self.get_parameter("covariance_x").value)
        self.cov_y = float(self.get_parameter("covariance_y").value)
        self.cov_yaw = float(self.get_parameter("covariance_yaw").value)

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.initialpose_topic, 10
        )

        # Relay: /reset_initialpose -> /initialpose
        self.create_subscription(
            PoseWithCovarianceStamped,
            reset_pose_topic,
            self._relay_pose,
            10,
        )

        # Service: publish default pose
        srv_name = self.get_parameter("service_name").value
        self.create_service(Trigger, srv_name, self._handle_reset)

        self.get_logger().info(
            f"Localization reset ready. Relay: {reset_pose_topic} -> {self.initialpose_topic}; "
            f"Trigger service: {srv_name}; default pose (x,y,yaw): {self.default_pose}"
        )

    # ------------------------------------------------------------------ #
    def _parse_pose(self, pose_list: List[float]):
        """Ensure pose is a 3-length list [x, y, yaw]."""
        if not isinstance(pose_list, list):
            self.get_logger().warn("default_pose_xy_yaw must be a list; using zeros.")
            return [0.0, 0.0, 0.0]
        vals = (pose_list + [0.0, 0.0, 0.0])[:3]
        try:
            return [float(v) for v in vals]
        except (TypeError, ValueError):
            self.get_logger().warn("default_pose_xy_yaw invalid; using zeros.")
            return [0.0, 0.0, 0.0]

    def _make_initialpose(self, x: float, y: float, yaw: float) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = self._yaw_to_quaternion(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Covariance in row-major 6x6
        cov = [0.0] * 36
        cov[0] = self.cov_x   # x
        cov[7] = self.cov_y   # y
        cov[35] = self.cov_yaw  # yaw
        msg.pose.covariance = cov
        return msg

    def _handle_reset(self, _req, _resp):
        pose = self._make_initialpose(*self.default_pose)
        self.publisher.publish(pose)
        self.get_logger().info(
            f"Published default /initialpose x={self.default_pose[0]:.2f}, "
            f"y={self.default_pose[1]:.2f}, yaw={math.degrees(self.default_pose[2]):.1f} deg"
        )
        return Trigger.Response(success=True, message="initialpose published")

    def _relay_pose(self, msg: PoseWithCovarianceStamped):
        if not msg.header.frame_id:
            msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Relayed /reset_initialpose -> /initialpose ({msg.header.frame_id})"
        )

    @staticmethod
    def _yaw_to_quaternion(yaw: float):
        half = yaw * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationResetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
