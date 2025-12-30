#!/usr/bin/env python3
import math
import random
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class RLAgentNode(Node):
    """
    RL-based local planner.
    Uses Torch policy if available, otherwise a safe heuristic.
    """

    def __init__(self):
        super().__init__("shobot_rl_agent")

        self.declare_parameter("model_path", "")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_speed", 0.2)
        self.declare_parameter("angular_speed", 0.4)

        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value

        self.policy = self._load_policy(self.get_parameter("model_path").value)

        self.latest_scan: Optional[LaserScan] = None
        self.latest_odom: Optional[Odometry] = None

        self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self._scan_cb,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self._odom_cb,
            10,
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            10,
        )

        self.create_timer(0.1, self._tick)  # 10 Hz

        self.get_logger().info("RL Agent (local planner) started")

    # --------------------------------------------------
    def _load_policy(self, model_path: str):
        if not model_path:
            self.get_logger().warn("No RL model provided → heuristic mode")
            return None

        path = Path(model_path)
        if not path.exists():
            self.get_logger().warn("RL model not found → heuristic mode")
            return None

        try:
            import torch
            policy = torch.jit.load(str(path))
            policy.eval()
            self.get_logger().info(f"Loaded RL policy from {path}")
            return policy
        except Exception as exc:
            self.get_logger().error(f"Failed to load RL model: {exc}")
            return None

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def _odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    def _tick(self):
        if not self.latest_scan or not self.latest_odom:
            return

        v, w = self._choose_action(self.latest_scan)
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    # --------------------------------------------------
    def _choose_action(self, scan: LaserScan) -> Tuple[float, float]:
        ranges = [r for r in scan.ranges if not math.isinf(r)]
        if not ranges:
            return 0.0, 0.0

        min_r = min(ranges)
        idx = scan.ranges.index(min_r)
        angle = scan.angle_min + idx * scan.angle_increment

        if min_r < 0.25:
            return 0.0, 0.0
        if min_r < 0.8:
            turn_dir = -1.0 if angle > 0 else 1.0
            return 0.0, turn_dir * self.angular_speed

        jitter = (random.random() - 0.5) * 0.1
        return self.linear_speed, jitter


def main():
    rclpy.init()
    node = RLAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
