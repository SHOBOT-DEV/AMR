#!/usr/bin/env python3
"""
Docking Manager for SHOBOT AMR
-----------------------------
Minimal docking controller using odometry distance.

Behavior:
- Drives forward at constant speed
- Stops when target distance is reached OR timeout occurs
- Exposes a Dock action server
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from shobot_docking.action import Dock


class DockingNode(Node):
    """Minimal, safe docking controller."""

    def __init__(self):
        super().__init__("shobot_docking")

        # ---------------- Parameters ----------------
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("dock_speed", 0.1)
        self.declare_parameter("dock_distance", 0.3)
        self.declare_parameter("timeout_sec", 30.0)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.dock_speed = float(self.get_parameter("dock_speed").value)
        self.dock_distance = float(self.get_parameter("dock_distance").value)
        self.timeout = Duration(
            seconds=float(self.get_parameter("timeout_sec").value)
        )

        # ---------------- Publishers / Subscribers ----------------
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.docked_pub = self.create_publisher(Bool, "/dock_status", 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)

        # ---------------- Action Server ----------------
        self.action_server = ActionServer(
            self,
            Dock,
            "dock",
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
        )

        # ---------------- State ----------------
        self.current_pose: Optional[Odometry] = None
        self.start_pose: Optional[Odometry] = None
        self.goal_active = False

        self.get_logger().info("SHOBOT DockingNode ready.")

    # ------------------------------------------------------------------
    # Action Callbacks
    # ------------------------------------------------------------------
    def goal_callback(self, goal_request):
        if self.goal_active:
            self.get_logger().warn("Rejecting dock goal: another goal is active.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Dock cancel requested.")
        self.stop()
        self.goal_active = False
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------
    # Odometry Callback
    # ------------------------------------------------------------------
    def odom_cb(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    # ------------------------------------------------------------------
    # Dock Execution
    # ------------------------------------------------------------------
    def execute_callback(self, goal_handle):
        self.get_logger().info("Dock execution started.")
        self.goal_active = True

        # --- Safety check ---
        if self.current_pose is None:
            self.get_logger().error("No odometry received. Aborting dock.")
            goal_handle.abort()
            self.goal_active = False
            return Dock.Result(success=False)

        self.start_pose = self.current_pose
        start_time = self.get_clock().now()

        rate = self.create_rate(10)  # 10 Hz loop

        while rclpy.ok():

            # ---- Cancel handling ----
            if goal_handle.is_cancel_requested:
                self.stop()
                goal_handle.canceled()
                self.docked_pub.publish(Bool(data=False))
                self.goal_active = False
                self.get_logger().info("Dock canceled.")
                return Dock.Result(success=False)

            # ---- Progress check ----
            if self.current_pose and self.start_pose:
                dist = self._distance(self.start_pose, self.current_pose)
                goal_handle.publish_feedback(Dock.Feedback(distance=dist))

                if dist >= self.dock_distance:
                    self.stop()
                    self.docked_pub.publish(Bool(data=True))
                    goal_handle.succeed()
                    self.goal_active = False
                    self.get_logger().info(
                        f"Dock successful (distance = {dist:.2f} m)"
                    )
                    return Dock.Result(success=True)

            # ---- Timeout ----
            if self.get_clock().now() - start_time > self.timeout:
                self.stop()
                goal_handle.abort()
                self.goal_active = False
                self.get_logger().warn("Dock aborted: timeout reached.")
                return Dock.Result(success=False)

            # ---- Motion ----
            self.drive_forward()
            rate.sleep()

        # ---- Fallback ----
        self.stop()
        self.goal_active = False
        return Dock.Result(success=False)

    # ------------------------------------------------------------------
    # Motion Helpers
    # ------------------------------------------------------------------
    def drive_forward(self):
        cmd = Twist()
        cmd.linear.x = self.dock_speed
        self.cmd_pub.publish(cmd)

    def stop(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def _distance(p1, p2):
        dx = p2.position.x - p1.position.x
        dy = p2.position.y - p1.position.y
        return math.hypot(dx, dy)


# ----------------------------------------------------------------------
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
