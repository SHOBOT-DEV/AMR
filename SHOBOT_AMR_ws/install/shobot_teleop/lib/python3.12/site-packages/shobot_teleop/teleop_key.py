#!/usr/bin/env python3
import sys
import termios
import tty
from select import select

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

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

space key, s : force stop

CTRL-C to quit
"""


def get_key(old_settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return key


def make_simple_profile(output, target, step):
    if target > output:
        output = min(target, output + step)
    elif target < output:
        output = max(target, output - step)
    return output


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


class TeleopNode(Node):
    def __init__(self):
        super().__init__("shobot_teleop")
        self.declare_parameter("model", "burger")
        self.declare_parameter("teleop_topic", "/joy/cmd_vel")
        self.declare_parameter("rate_hz", 20.0)

        topic = self.get_parameter("teleop_topic").value
        self.publisher_ = self.create_publisher(Twist, topic, 10)
        self.model = self.get_parameter("model").value

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.control_linear = 0.0
        self.control_angular = 0.0

        self.old_settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info(f"Publishing Twist on {topic} (model={self.model})")
        print(MSG)
        try:
            self.rate_period = 1.0 / float(self.get_parameter("rate_hz").value)
        except (ValueError, ZeroDivisionError):
            self.rate_period = 0.05
            self.get_logger().warn("Invalid rate_hz parameter; defaulting to 20 Hz.")
        self.rate_timer = self.create_timer(self.rate_period, lambda: None)

    def spin_keyboard(self):
        status = 0
        try:
            while rclpy.ok():
                key = get_key(self.old_settings)
                if key == "w":
                    self.target_linear = limit_linear(self.model, self.target_linear + LIN_VEL_STEP_SIZE)
                elif key == "x":
                    self.target_linear = limit_linear(self.model, self.target_linear - LIN_VEL_STEP_SIZE)
                elif key == "a":
                    self.target_angular = limit_angular(self.model, self.target_angular + ANG_VEL_STEP_SIZE)
                elif key == "d":
                    self.target_angular = limit_angular(self.model, self.target_angular - ANG_VEL_STEP_SIZE)
                elif key in (" ", "s"):
                    self.target_linear = 0.0
                    self.target_angular = 0.0
                    self.control_linear = 0.0
                    self.control_angular = 0.0
                else:
                    if key == "\x03":
                        break

                status += 1
                if status == 20:
                    print(MSG)
                    status = 0

                twist = Twist()
                self.control_linear = make_simple_profile(
                    self.control_linear, self.target_linear, LIN_VEL_STEP_SIZE / 2.0
                )
                self.control_angular = make_simple_profile(
                    self.control_angular, self.target_angular, ANG_VEL_STEP_SIZE / 2.0
                )

                twist.linear.x = self.control_linear
                twist.angular.z = self.control_angular
                self.publisher_.publish(twist)
                # Sleep roughly to the requested rate
                rclpy.spin_once(self, timeout_sec=self.rate_period)
        finally:
            twist = Twist()
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    node.spin_keyboard()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
