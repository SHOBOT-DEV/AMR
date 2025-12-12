#!/usr/bin/env python3
"""Battery monitoring node: republish status and low-battery flag."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String


class BatteryMonitor(Node):
    """Monitor battery state, republish status, and emit low-battery alarms."""

    def __init__(self):
        super().__init__("shobot_battery_monitor")
        self.declare_parameter("battery_topic", "/battery_state")
        self.declare_parameter("status_topic", "/battery_status")
        self.declare_parameter("low_topic", "/battery_low")
        self.declare_parameter("low_threshold", 0.2)  # 20%
        self.declare_parameter("critical_threshold", 0.1)  # 10%

        battery_topic = self.get_parameter("battery_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.low_topic = self.get_parameter("low_topic").value
        self.low_threshold = float(self.get_parameter("low_threshold").value)
        self.critical_threshold = float(self.get_parameter("critical_threshold").value)

        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.low_pub = self.create_publisher(Bool, self.low_topic, 10)
        self.create_subscription(BatteryState, battery_topic, self.battery_cb, 10)

        self.get_logger().info(
            f"Monitoring battery on {battery_topic}; publishing {self.status_topic} and {self.low_topic}"
        )

    def battery_cb(self, msg: BatteryState):
        percentage = msg.percentage if msg.percentage is not None else 0.0
        charging = msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
        current = msg.current if msg.current is not None else 0.0
        temperature = msg.temperature if msg.temperature is not None else 0.0

        status_text = (
            f"battery: {percentage*100:.1f}%, "
            f"charging: {charging}, current: {current:.2f}A, temp: {temperature:.1f}C"
        )
        self.status_pub.publish(String(data=status_text))

        low = percentage <= self.low_threshold
        critical = percentage <= self.critical_threshold
        self.low_pub.publish(Bool(data=low or critical))

        if critical:
            self.get_logger().warn(f"Battery critically low ({percentage*100:.1f}%). Consider docking.")
        elif low:
            self.get_logger().info(f"Battery low ({percentage*100:.1f}%).")


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
