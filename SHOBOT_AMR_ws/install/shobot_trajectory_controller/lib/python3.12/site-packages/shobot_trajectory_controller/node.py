#!/usr/bin/env python3
"""
Trajectory Controller
===========================================================
Smooths incoming /cmd_vel_raw using acceleration limits,
ensuring safe and realistic robot motion.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TrajectoryController(Node):
    """Applies acceleration limiting to cmd_vel commands."""

    def __init__(self):
        super().__init__("shobot_trajectory_controller")

        # ---------------- Parameters ----------------
        self.declare_parameter("input_topic", "/cmd_vel_raw")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("max_linear_accel", 0.5)
        self.declare_parameter("max_angular_accel", 1.0)
        self.declare_parameter("instant_stop", True)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.instant_stop = self.get_parameter("instant_stop").value

        rate_hz = float(self.get_parameter("rate_hz").value)
        self.dt = 1.0 / rate_hz

        # Acceleration limits
        self.max_lin_accel = float(self.get_parameter("max_linear_accel").value)
        self.max_ang_accel = float(self.get_parameter("max_angular_accel").value)

        # State buffers
        self.target = Twist()
        self.current = Twist()

        # Publishers / Subscribers
        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        self.create_subscription(Twist, self.input_topic, self.cmd_cb, 10)

        # Run smoother
        self.create_timer(self.dt, self.update)

        self.get_logger().info(
            f"Trajectory smoothing active: {self.input_topic} -> {self.output_topic}  "
            f"at {rate_hz} Hz | lin_accel={self.max_lin_accel} m/s² | "
            f"ang_accel={self.max_ang_accel} rad/s²"
        )

    # ---------------------------------------------------------------------
    def cmd_cb(self, msg: Twist):
        """Receive new target velocities."""
        self.target = msg

    # ---------------------------------------------------------------------
    def update(self):
        """Apply acceleration limits and publish result."""

        # ---------------- Linear Acceleration ----------------
        self.current.linear.x = self._ramp(
            self.current.linear.x, self.target.linear.x, self.max_lin_accel
        )

        # Optional for holonomic robots (disabled for differential robots)
        self.current.linear.y = self._ramp(
            self.current.linear.y, self.target.linear.y, self.max_lin_accel
        )

        # ---------------- Angular Acceleration ----------------
        self.current.angular.z = self._ramp(
            self.current.angular.z, self.target.angular.z, self.max_ang_accel
        )

        # Publish smoothed command
        self.pub.publish(self.current)

    # ---------------------------------------------------------------------
    def _ramp(self, current: float, target: float, accel_limit: float) -> float:
        """Smooth value 'current' toward 'target' obeying acceleration limits."""

        # Allow INSTANT stopping if sudden zero command received
        if self.instant_stop and abs(target) < 1e-4:
            return 0.0

        step = accel_limit * self.dt

        if target > current:
            return min(target, current + step)
        else:
            return max(target, current - step)


# ======================================================================
def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
