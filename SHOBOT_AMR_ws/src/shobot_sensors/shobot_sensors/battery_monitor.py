#!/usr/bin/env python3
"""
Battery Monitoring Node for SHOBOT
=================================

- Subscribes to sensor_msgs/BatteryState
- Publishes JSON battery status
- Publishes low/critical battery alarm
"""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String


class BatteryMonitor(Node):
    """Monitor battery state and publish status + alarms."""

    def __init__(self):
        super().__init__("shobot_battery_monitor")

        # ---------------- Parameters ----------------
        self.declare_parameter("battery_topic", "/battery_state")
        self.declare_parameter("status_topic", "/battery_status")
        self.declare_parameter("low_topic", "/battery_low")
        self.declare_parameter("low_threshold", 0.2)       # 20%
        self.declare_parameter("critical_threshold", 0.1)  # 10%

        battery_topic = self.get_parameter("battery_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.low_topic = self.get_parameter("low_topic").value

        self.low_threshold = float(self.get_parameter("low_threshold").value)
        self.critical_threshold = float(
            self.get_parameter("critical_threshold").value
        )

        # ---------------- QoS ----------------
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # ---------------- Publishers ----------------
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.low_pub = self.create_publisher(Bool, self.low_topic, 10)

        # ---------------- Subscriber ----------------
        self.create_subscription(
            BatteryState,
            battery_topic,
            self.battery_cb,
            qos,
        )

        self.last_state = None

        self.get_logger().info(
            f"Battery monitor started → input={battery_topic}, "
            f"status={self.status_topic}, alarm={self.low_topic}"
        )

    # -------------------------------------------------
    def battery_cb(self, msg: BatteryState):
        """Handle battery updates."""

        percentage = msg.percentage
        valid_percentage = 0.0 <= percentage <= 1.0

        charging = (
            msg.power_supply_status
            == BatteryState.POWER_SUPPLY_STATUS_CHARGING
        )

        current = msg.current
        temperature = msg.temperature

        # Determine state
        if not valid_percentage:
            state = "UNKNOWN"
        elif percentage <= self.critical_threshold:
            state = "CRITICAL"
        elif percentage <= self.low_threshold:
            state = "LOW"
        else:
            state = "OK"

        # Publish JSON status
        status_json = {
            "timestamp_ns": self.get_clock().now().nanoseconds,
            "percentage": percentage if valid_percentage else None,
            "charging": charging,
            "current": current,
            "temperature": temperature,
            "state": state,
        }

        self.status_pub.publish(String(data=json.dumps(status_json)))

        # Publish alarm
        alarm = state in ("LOW", "CRITICAL")
        self.low_pub.publish(Bool(data=alarm))

        # Log only on state change
        if state != self.last_state:
            self.last_state = state
            if state == "CRITICAL":
                self.get_logger().warn(
                    f"Battery CRITICAL ({percentage*100:.1f}%). Dock immediately."
                )
            elif state == "LOW":
                self.get_logger().info(
                    f"Battery LOW ({percentage*100:.1f}%)."
                )
            elif state == "OK":
                self.get_logger().info("Battery OK.")
            else:
                self.get_logger().warn("Battery percentage UNKNOWN.")


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
