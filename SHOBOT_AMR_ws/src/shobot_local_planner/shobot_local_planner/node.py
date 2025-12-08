#!/usr/bin/env python3
"""Very simple local planner that follows waypoints as a velocity stream."""
from collections import deque
from typing import Deque, List

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
from rclpy.node import Node


class LocalPlanner(Node):
    """Consumes a nav_msgs/Path and outputs a smoothed Twist towards the next waypoint."""

    def __init__(self):
        super().__init__("shobot_local_planner")

        self.declare_parameter("path_topic", "/plan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
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

        self.path: Deque[PoseStamped] = deque()
        self.current_pose: PoseStamped | None = None

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Path, self.path_topic, self.path_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)

        self.timer = self.create_timer(0.05, self.control_step)
        self.get_logger().info(
            f"Local planner listening on {self.path_topic}, publishing {self.cmd_vel_topic}"
        )

    def path_cb(self, msg: Path):
        self.path = deque(msg.poses)
        self.get_logger().info(f"Received path with {len(self.path)} poses.")

    def odom_cb(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.current_pose = pose

    def control_step(self):
        if self.current_pose is None or not self.path:
            return

        # Drop waypoints behind us
        while self.path and self._distance(self.current_pose, self.path[0]) < self.lookahead / 2.0:
            self.path.popleft()

        if not self.path:
            self.cmd_pub.publish(Twist())
            return

        target = self.path[0]
        cmd = self._compute_cmd(self.current_pose, target)
        self.cmd_pub.publish(cmd)

    def _compute_cmd(self, current: PoseStamped, target: PoseStamped) -> Twist:
        dx = target.pose.position.x - current.pose.position.x
        dy = target.pose.position.y - current.pose.position.y
        dist = max(1e-3, (dx**2 + dy**2) ** 0.5)

        # Simple heading control towards target
        heading = math.atan2(dy, dx)
        yaw = self._yaw_from_quat(current.pose.orientation)
        heading_error = self._normalize_angle(heading - yaw)

        cmd = Twist()
        cmd.linear.x = min(self.max_linear, dist)
        cmd.angular.z = max(-self.max_angular, min(self.max_angular, heading_error * 1.5))
        return cmd

    @staticmethod
    def _distance(p1: PoseStamped, p2: PoseStamped) -> float:
        dx = p2.pose.position.x - p1.pose.position.x
        dy = p2.pose.position.y - p1.pose.position.y
        return (dx**2 + dy**2) ** 0.5

    @staticmethod
    def _yaw_from_quat(q) -> float:
        import math

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(a: float) -> float:
        import math

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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
