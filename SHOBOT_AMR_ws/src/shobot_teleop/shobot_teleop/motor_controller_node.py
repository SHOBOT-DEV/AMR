#!/usr/bin/env python3
"""
Keyboard Teleop for SHOBOT
=====================================================
Publishes cmd_vel for manual robot control.

Keys:
  w / x  : increase / decrease linear velocity
  a / d  : increase / decrease angular velocity
  space or s : stop
  CTRL+C : quit
"""

import sys
import termios
import tty
from select import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ---------------- Velocity Limits ----------------
MAX_LIN_VEL = 0.8   # m/s
MAX_ANG_VEL = 0.6   # rad/s

LIN_STEP = 0.02
ANG_STEP = 0.1

HELP = """
SHOBOT TELEOP
-------------------------
w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space or s : stop
CTRL+C : quit
"""


def get_key(old_settings):
    """Non-blocking keyboard read."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else None
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return key


class TeleopNode(Node):
    """Keyboard teleoperation node."""

    def __init__(self):
        super().__init__("shobot_teleop_keyboard")

        # ---------------- Parameters ----------------
        self.declare_parameter("topic", "/cmd_vel/teleop")
        self.declare_parameter("rate_hz", 20.0)

        topic = self.get_parameter("topic").value
        rate_hz = float(self.get_parameter("rate_hz").value)

        # ---------------- Publisher ----------------
        self.pub = self.create_publisher(Twist, topic, 10)

        # ---------------- State ----------------
        self.target_lin = 0.0
        self.target_ang = 0.0

        # Save terminal state
        if not sys.stdin.isatty():
            self.get_logger().error("Teleop requires a TTY (run in a terminal).")
            raise RuntimeError("No TTY available")

        self.old_settings = termios.tcgetattr(sys.stdin)
        print(HELP)

        # Timer for periodic publishing
        self.timer = self.create_timer(1.0 / rate_hz, self.update)

        self.get_logger().info(
            f"Keyboard teleop started → publishing to {topic} at {rate_hz} Hz"
        )

    # -------------------------------------------------
    def update(self):
        """Main control loop."""
        key = get_key(self.old_settings)

        if key == "w":
            self.target_lin = min(self.target_lin + LIN_STEP, MAX_LIN_VEL)
        elif key == "x":
            self.target_lin = max(self.target_lin - LIN_STEP, -MAX_LIN_VEL)
        elif key == "a":
            self.target_ang += ANG_STEP
        elif key == "d":
            self.target_ang -= ANG_STEP
        elif key in (" ", "s"):
            self.target_lin = 0.0
            self.target_ang = 0.0
        elif key == "\x03":  # CTRL+C
            raise KeyboardInterrupt

        # Clamp angular velocity
        self.target_ang = max(
            min(self.target_ang, MAX_ANG_VEL),
            -MAX_ANG_VEL,
        )

        twist = Twist()
        twist.linear.x = self.target_lin
        twist.angular.z = self.target_ang

        self.pub.publish(twist)

    # -------------------------------------------------
    def destroy_node(self):
        """Stop robot and restore terminal settings."""
        stop = Twist()
        self.pub.publish(stop)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()


# =================================================
def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
