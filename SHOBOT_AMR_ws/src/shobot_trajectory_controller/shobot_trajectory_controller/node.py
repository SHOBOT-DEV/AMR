#!/usr/bin/env python3
"""
Trajectory Controller
===========================================================
Smooths incoming /cmd_vel_raw using acceleration limits,
ensuring safe and realistic robot motion.

Typical placement in stack:
  cmd_vel sources → twist_mux → trajectory_controller → motor driver
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TrajectoryController(Node):
    """Applies acceleration limiting to velocity commands."""

    def __init__(self):
        super().__init__("shobot_trajectory_controller")

        # ---------------- Parameters ----------------
        self.declare_parameter("input_topic", "/cmd_vel_raw")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("rate_hz", 30.0)

        self.declare_parameter("max_linear_accel", 0.5)     # m/s^2
        self.declare_parameter("max_angular_accel", 1.0)    # rad/s^2

        self.declare_parameter("instant_stop", True)

        # ---------------- Load Parameters ----------------
        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.instant_stop = bool(self.get_parameter("instant_stop").value)

        rate_hz = float(self.get_parameter("rate_hz").value)
        self.dt = 1.0 / rate_hz

        self.max_lin_accel = float(self.get_parameter("max_linear_accel").value)
        self.max_ang_accel = float(self.get_parameter("max_angular_accel").value)

        # ---------------- State ----------------
        self.target = Twist()
        self.current = Twist()

        # ---------------- ROS Interfaces ----------------
        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        self.create_subscription(
            Twist,
            self.input_topic,
            self.cmd_cb,
            10,
        )

        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info(
            f"Trajectory Controller started\n"
            f"Input topic  : {self.input_topic}\n"
            f"Output topic : {self.output_topic}\n"
            f"Rate         : {rate_hz} Hz\n"
            f"Lin accel    : {self.max_lin_accel} m/s²\n"
            f"Ang accel    : {self.max_ang_accel} rad/s²\n"
            f"Instant stop: {self.instant_stop}"
        )

    # ------------------------------------------------------------------
    def cmd_cb(self, msg: Twist):
        """Receive new target velocity."""
        self.target = msg

    # ------------------------------------------------------------------
    def update(self):
        """Apply acceleration limits and publish smoothed command."""

        # -------- Linear X --------
        self.current.linear.x = self._ramp(
            self.current.linear.x,
            self.target.linear.x,
            self.max_lin_accel,
        )

        # -------- Linear Y (holonomic robots only) --------
        self.current.linear.y = self._ramp(
            self.current.linear.y,
            self.target.linear.y,
            self.max_lin_accel,
        )

        # -------- Angular Z --------
        self.current.angular.z = self._ramp(
            self.current.angular.z,
            self.target.angular.z,
            self.max_ang_accel,
        )

        self.pub.publish(self.current)

    # ------------------------------------------------------------------
    def _ramp(self, current: float, target: float, accel_limit: float) -> float:
        """
        Smoothly move 'current' toward 'target' while respecting
        acceleration limits.
        """

        # Allow instant stop on zero command (safety behavior)
        if self.instant_stop and abs(target) < 1e-4:
            return 0.0

        step = accel_limit * self.dt

        if target > current:
            return min(target, current + step)
        else:
            return max(target, current - step)


# =====================================================================
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
