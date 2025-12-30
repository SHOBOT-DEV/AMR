#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SpeedModulationNode(Node):
    """
    Modulates linear velocity only.
    Steering is untouched.
    """

    def __init__(self):
        super().__init__("shobot_rl_speed")

        self.declare_parameter("input_cmd_topic", "/cmd_vel_raw")
        self.declare_parameter("output_cmd_topic", "/cmd_vel")

        self.latest_cmd = None

        self.create_subscription(
            Twist,
            self.get_parameter("input_cmd_topic").value,
            self._cmd_cb,
            10,
        )
        self.cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter("output_cmd_topic").value,
            10,
        )

        self.create_timer(0.05, self._tick)  # 20 Hz

        self.get_logger().info("RL Speed Modulation node started")

    def _cmd_cb(self, msg: Twist):
        self.latest_cmd = msg

    def _tick(self):
        if not self.latest_cmd:
            return

        out = Twist()
        out.linear.x = 0.8 * self.latest_cmd.linear.x
        out.angular.z = self.latest_cmd.angular.z
        self.cmd_pub.publish(out)


def main():
    rclpy.init()
    node = SpeedModulationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
