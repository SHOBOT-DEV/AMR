#!/usr/bin/env python3
"""
SHOBOT Local Planner
======================================================
A lightweight pure pursuit–style local planner that
converts a Path (nav_msgs/Path) into a velocity stream
(geometry_msgs/Twist).
"""

from collections import deque
from typing import Deque, Optional
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry


class LocalPlanner(Node):
    """Consumes a nav_msgs/Path and outputs Twist towards next waypoint."""

    def __init__(self):
        super().__init__("shobot_local_planner")

        # ---------------- Parameters ----------------
        self.declare_parameter("path_topic", "/plan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel/nav")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("lookahead_distance", 0.6)
        self.declare_parameter("max_linear", 0.4)
        self.declare_parameter("max_angular", 1.2)

        self.path_topic = self.get_parameter("path_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.lookahead = float(self.get_parameter("lookahead_distance").value)
        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)

        # ---------------- State ----------------
        self.path: Deque[PoseStamped] = deque()
        self.current_pose: Optional[PoseStamped] = None

        # ---------------- ROS interfaces ----------------
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Path, self.path_topic, self.path_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)

        self.timer = self.create_timer(0.05, self.control_step)

        self.get_logger().info(
            f"Local Planner: listening on {self.path_topic}, publishing {self.cmd_vel_topic}"
        )

    # ===================================================================
    # Callbacks
    # ===================================================================
    def path_cb(self, msg: Path):
        """Receive a full path and reset internal buffer."""
        self.path = deque(msg.poses)
        self.get_logger().info(f"Received path with {len(self.path)} waypoints.")

    def odom_cb(self, msg: Odometry):
        """Convert odometry to PoseStamped for easier math."""
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.current_pose = pose

    # ===================================================================
    # Control Loop
    # ===================================================================
    def control_step(self):
        if self.current_pose is None or not self.path:
            return

        # Remove waypoints behind or very close
        while self.path:
            dist = self._distance(self.current_pose, self.path[0])
            if dist < (self.lookahead * 0.5):
                self.path.popleft()
            else:
                break

        if not self.path:
            # Completed path
            self.cmd_pub.publish(Twist())
            self.get_logger().info("Path completed — stopping robot.")
            return

        target = self._choose_lookahead_point()

        cmd = self._compute_cmd(self.current_pose, target)
        self.cmd_pub.publish(cmd)

    # ===================================================================
    # Waypoint / Lookahead selection
    # ===================================================================
    def _choose_lookahead_point(self) -> PoseStamped:
        """
        Select the waypoint at least `lookahead_distance` away.
        If none found, use last waypoint.
        """
        if not self.path:
            return None

        for wp in self.path:
            if self._distance(self.current_pose, wp) >= self.lookahead:
                return wp

        return self.path[-1]

    # ===================================================================
    # Control computation
    # ===================================================================
    def _compute_cmd(self, current: PoseStamped, target: PoseStamped) -> Twist:
        dx = target.pose.position.x - current.pose.position.x
        dy = target.pose.position.y - current.pose.position.y
        dist = max(1e-3, math.sqrt(dx * dx + dy * dy))

        heading = math.atan2(dy, dx)
        yaw = self._yaw_from_quat(current.pose.orientation)
        heading_error = self._normalize_angle(heading - yaw)

        cmd = Twist()

        # --- Linear speed (distance based with soft stopping) ---
        cmd.linear.x = min(self.max_linear, dist)

        # --- Angular speed (steering) ---
        cmd.angular.z = max(-self.max_angular, min(self.max_angular, heading_error * 2.0))

        # If angle is too large, slow down for turning
        if abs(heading_error) > 1.0:
            cmd.linear.x *= 0.4

        return cmd

    # ===================================================================
    # Utilities
    # ===================================================================
    @staticmethod
    def _distance(p1: PoseStamped, p2: PoseStamped) -> float:
        dx = p2.pose.position.x - p1.pose.position.x
        dy = p2.pose.position.y - p1.pose.position.y
        return math.sqrt(dx * dx + dy * dy)

    @staticmethod
    def _yaw_from_quat(q) -> float:
        """Quaternion → yaw."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(a: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
