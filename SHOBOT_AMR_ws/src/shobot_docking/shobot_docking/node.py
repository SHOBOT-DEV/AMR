#!/usr/bin/env python3
"""Simple docking manager: listens for a Dock action and publishes cmd_vel."""
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool
from tf_transformations import euler_from_quaternion

from shobot_docking.action import Dock  # custom action: goal bool start; result bool success


class DockingNode(Node):
    """Minimal docking controller: drive straight until distance threshold or timeout."""

    def __init__(self):
        super().__init__("shobot_docking")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("dock_speed", 0.1)
        self.declare_parameter("dock_distance", 0.3)
        self.declare_parameter("timeout_sec", 30.0)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.dock_speed = float(self.get_parameter("dock_speed").value)
        self.dock_distance = float(self.get_parameter("dock_distance").value)
        self.timeout = Duration(seconds=float(self.get_parameter("timeout_sec").value))

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.docked_pub = self.create_publisher(Bool, "/dock_status", 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)

        self.action_server = ActionServer(
            self,
            Dock,
            "dock",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.current_pose = None
        self.start_pose = None
        self.goal_active = False

        self.get_logger().info("shobot_docking action server ready.")

    def goal_callback(self, goal_request):
        if self.goal_active:
            self.get_logger().warn("Dock goal rejected: already active.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Dock goal cancel requested.")
        self.stop()
        self.goal_active = False
        return CancelResponse.ACCEPT

    def odom_cb(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def execute_callback(self, goal_handle):
        self.get_logger().info("Dock goal received.")
        self.goal_active = True
        self.start_pose = self.current_pose
        start_time = self.get_clock().now()

        success = False

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.stop()
                self.goal_active = False
                self.get_logger().info("Dock goal canceled.")
                return Dock.Result(success=False)

            if self.current_pose and self.start_pose:
                dist = self._distance(self.start_pose, self.current_pose)
                if dist >= self.dock_distance:
                    success = True
                    self.stop()
                    self.docked_pub.publish(Bool(data=True))
                    goal_handle.succeed()
                    self.goal_active = False
                    self.get_logger().info("Dock goal succeeded.")
                    return Dock.Result(success=True)

            if self.get_clock().now() - start_time > self.timeout:
                self.stop()
                self.goal_active = False
                self.get_logger().warn("Dock goal timed out.")
                goal_handle.abort()
                return Dock.Result(success=False)

            self.drive_forward()
            rclpy.spin_once(self, timeout_sec=0.1)

        self.stop()
        self.goal_active = False
        return Dock.Result(success=False)

    def drive_forward(self):
        cmd = Twist()
        cmd.linear.x = self.dock_speed
        self.cmd_pub.publish(cmd)

    def stop(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def _distance(p1, p2) -> float:
        dx = p2.position.x - p1.position.x
        dy = p2.position.y - p1.position.y
        return math.sqrt(dx * dx + dy * dy)


def main(args=None):
    rclpy.init(args=args)
    node = DockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
