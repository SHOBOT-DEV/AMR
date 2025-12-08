#!/usr/bin/env python3
"""Trajectory controller that smooths incoming cmd_vel with accel limits."""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TrajectoryController(Node):
    """Rate-limit cmd_vel to respect acceleration limits before publishing."""

    def __init__(self):
        super().__init__("shobot_trajectory_controller")
        self.declare_parameter("input_topic", "/cmd_vel_raw")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("max_linear_accel", 0.5)
        self.declare_parameter("max_angular_accel", 1.0)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.dt = 1.0 / float(self.get_parameter("rate_hz").value)
        self.max_lin_accel = float(self.get_parameter("max_linear_accel").value)
        self.max_ang_accel = float(self.get_parameter("max_angular_accel").value)

        self.target = Twist()
        self.current = Twist()

        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        self.create_subscription(Twist, self.input_topic, self.cmd_cb, 10)
        self.create_timer(self.dt, self.update)
        self.get_logger().info(
            f"Smoothing {self.input_topic} -> {self.output_topic} at {1/self.dt:.1f} Hz"
        )

    def cmd_cb(self, msg: Twist):
        self.target = msg

    def update(self):
        self.current.linear.x = self._ramp(self.current.linear.x, self.target.linear.x, self.max_lin_accel)
        self.current.angular.z = self._ramp(self.current.angular.z, self.target.angular.z, self.max_ang_accel)
        self.pub.publish(self.current)

    def _ramp(self, current: float, target: float, accel_limit: float) -> float:
        step = accel_limit * self.dt
        if target > current:
            return min(target, current + step)
        return max(target, current - step)


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
