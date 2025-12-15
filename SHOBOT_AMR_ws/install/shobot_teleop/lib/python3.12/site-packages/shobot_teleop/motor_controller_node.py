#!/usr/bin/env python3
"""
Keyboard Teleop for SHOBOT
=====================================================
Publishes cmd_vel for manual robot control.
"""

import sys
import termios
import tty
from select import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Velocity limits
MAX_LIN_VEL = 0.8
MAX_ANG_VEL = 0.6

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
    tty.setraw(sys.stdin.fileno())
    r, _, _ = select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if r else None
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return key


class TeleopNode(Node):
    def __init__(self):
        super().__init__("shobot_teleop_keyboard")

        self.declare_parameter("topic", "/cmd_vel/teleop")
        self.declare_parameter("rate_hz", 20.0)

        topic = self.get_parameter("topic").value
        rate = float(self.get_parameter("rate_hz").value)

        self.pub = self.create_publisher(Twist, topic, 10)

        self.target_lin = 0.0
        self.target_ang = 0.0

        # Save terminal state
        self.old_settings = termios.tcgetattr(sys.stdin)
        print(HELP)

        # Timer for repeated publishing
        self.timer = self.create_timer(1.0 / rate, self.update)

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

        twist = Twist()
        twist.linear.x = self.target_lin
        twist.angular.z = self.target_ang

        self.pub.publish(twist)

    def destroy_node(self):
        """Restore terminal + stop robot."""
        stop_cmd = Twist()
        self.pub.publish(stop_cmd)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
