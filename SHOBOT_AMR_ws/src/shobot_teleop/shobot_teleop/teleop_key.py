#!/usr/bin/env python3
"""
Keyboard Teleop for SHOBOT (ROS 2)
=================================
Publishes geometry_msgs/Twist based on keyboard input.
Supports model-based velocity limits and smooth profiling.
"""

import sys
import termios
import tty
from select import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ---------------- Velocity Limits ----------------
BURGER_MAX_LIN_VEL = 0.8
BURGER_MAX_ANG_VEL = 0.6

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

MSG = """
Control Your SHOBOT!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space or s : force stop
CTRL+C : quit
"""


# ---------------- Utility Functions ----------------
def get_key(old_settings):
    """Read one key press (non-blocking)."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return key


def constrain(value, low, high):
    return max(min(value, high), low)


def limit_linear(model, vel):
    if model == "burger":
        return constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    if model in ("waffle", "waffle_pi"):
        return constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    return constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)


def limit_angular(model, vel):
    if model == "burger":
        return constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    if model in ("waffle", "waffle_pi"):
        return constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    return constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)


def make_simple_profile(current, target, step):
    """Smooth velocity change."""
    if target > current:
        return min(target, current + step)
    if target < current:
        return max(target, current - step)
    return current


# ---------------- Teleop Node ----------------
class TeleopNode(Node):
    def __init__(self):
        super().__init__("shobot_teleop_keyboard")

        self.declare_parameter("model", "burger")
        self.declare_parameter("teleop_topic", "/cmd_vel/teleop")
        self.declare_parameter("rate_hz", 20.0)

        if not sys.stdin.isatty():
            self.get_logger().error(
                "Keyboard teleop requires a TTY (run in a terminal)."
            )
            raise SystemExit(1)

        self.model = self.get_parameter("model").value
        topic = self.get_parameter("teleop_topic").value
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.dt = 1.0 / rate_hz if rate_hz > 0 else 0.05

        self.publisher = self.create_publisher(Twist, topic, 10)

        # State
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.control_linear = 0.0
        self.control_angular = 0.0

        self.old_settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info(
            f"Keyboard teleop started → topic={topic}, model={self.model}, rate={rate_hz} Hz"
        )
        print(MSG)

    # -------------------------------------------------
    def spin_keyboard(self):
        """Main keyboard loop."""
        status = 0
        try:
            while rclpy.ok():
                key = get_key(self.old_settings)

                if key == "w":
                    self.target_linear = limit_linear(
                        self.model, self.target_linear + LIN_VEL_STEP_SIZE
                    )
                elif key == "x":
                    self.target_linear = limit_linear(
                        self.model, self.target_linear - LIN_VEL_STEP_SIZE
                    )
                elif key == "a":
                    self.target_angular = limit_angular(
                        self.model, self.target_angular + ANG_VEL_STEP_SIZE
                    )
                elif key == "d":
                    self.target_angular = limit_angular(
                        self.model, self.target_angular - ANG_VEL_STEP_SIZE
                    )
                elif key in (" ", "s"):
                    self.target_linear = 0.0
                    self.target_angular = 0.0
                    self.control_linear = 0.0
                    self.control_angular = 0.0
                elif key == "\x03":  # CTRL+C
                    break

                # Smooth output
                self.control_linear = make_simple_profile(
                    self.control_linear,
                    self.target_linear,
                    LIN_VEL_STEP_SIZE / 2.0,
                )
                self.control_angular = make_simple_profile(
                    self.control_angular,
                    self.target_angular,
                    ANG_VEL_STEP_SIZE / 2.0,
                )

                twist = Twist()
                twist.linear.x = self.control_linear
                twist.angular.z = self.control_angular
                self.publisher.publish(twist)

                status += 1
                if status == 20:
                    print(MSG)
                    status = 0

                rclpy.spin_once(self, timeout_sec=self.dt)

        finally:
            # Always stop robot + restore terminal
            self.publisher.publish(Twist())
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


# ---------------- Main ----------------
def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        node.spin_keyboard()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
