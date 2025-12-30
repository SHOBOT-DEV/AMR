#!/usr/bin/env python3
"""
Robot Diagnostics Node for SHOBOT AMR
=========================================================
Publishes diagnostic_msgs/DiagnosticArray on /diagnostics
summarizing robot health and subsystem status.
"""

from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

from std_msgs.msg import Float32, String, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


@dataclass
class SignalState:
    last_value: Optional[float] = None
    last_text: Optional[str] = None
    last_seen: Optional[Time] = None


class RobotDiagnosticsNode(Node):
    """Aggregates health signals into /diagnostics."""

    def __init__(self):
        super().__init__("shobot_robot_diagnostics")

        # ---------------- Parameters ----------------
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("timeout_sec", 2.0)

        self.declare_parameter("motor_temp_topic", "/motor/temperature")
        self.declare_parameter("motor_temp_warn", 70.0)
        self.declare_parameter("motor_temp_error", 85.0)

        self.declare_parameter("imu_error_topic", "/imu/errors")
        self.declare_parameter("encoder_error_topic", "/encoders/errors")

        self.declare_parameter("camera_status_topic", "/camera/status")
        self.declare_parameter("camera_status_type", "string")  # string | bool

        self.declare_parameter("network_quality_topic", "/network/quality")
        self.declare_parameter("network_warn", 40.0)
        self.declare_parameter("network_error", 15.0)

        self.publish_rate = max(
            float(self.get_parameter("publish_rate").value), 0.1
        )
        self.timeout_sec = max(
            float(self.get_parameter("timeout_sec").value), 0.1
        )

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        # ---------------- State ----------------
        self.motor_state = SignalState()
        self.imu_state = SignalState()
        self.encoder_state = SignalState()
        self.camera_state = SignalState()
        self.network_state = SignalState()

        # ---------------- Subscriptions ----------------
        self.create_subscription(
            Float32,
            self.get_parameter("motor_temp_topic").value,
            self._motor_cb,
            qos,
        )
        self.create_subscription(
            String,
            self.get_parameter("imu_error_topic").value,
            self._imu_cb,
            qos,
        )
        self.create_subscription(
            String,
            self.get_parameter("encoder_error_topic").value,
            self._encoder_cb,
            qos,
        )

        camera_topic = self.get_parameter("camera_status_topic").value
        cam_type = self.get_parameter("camera_status_type").value.lower()

        if cam_type == "bool":
            self.create_subscription(Bool, camera_topic, self._camera_bool_cb, qos)
        else:
            self.create_subscription(String, camera_topic, self._camera_cb, qos)

        self.create_subscription(
            Float32,
            self.get_parameter("network_quality_topic").value,
            self._network_cb,
            qos,
        )

        # ---------------- Publisher ----------------
        self.pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(1.0 / self.publish_rate, self._publish_diag)

        self.get_logger().info("Robot diagnostics node started.")

    # ------------------------------------------------------------------
    def _stamp(self):
        return self.get_clock().now()

    # Callbacks
    def _motor_cb(self, msg: Float32):
        self.motor_state.last_value = float(msg.data)
        self.motor_state.last_seen = self._stamp()

    def _imu_cb(self, msg: String):
        self.imu_state.last_text = msg.data
        self.imu_state.last_seen = self._stamp()

    def _encoder_cb(self, msg: String):
        self.encoder_state.last_text = msg.data
        self.encoder_state.last_seen = self._stamp()

    def _camera_cb(self, msg: String):
        self.camera_state.last_text = msg.data
        self.camera_state.last_seen = self._stamp()

    def _camera_bool_cb(self, msg: Bool):
        self.camera_state.last_text = "ok" if msg.data else "fault"
        self.camera_state.last_seen = self._stamp()

    def _network_cb(self, msg: Float32):
        self.network_state.last_value = float(msg.data)
        self.network_state.last_seen = self._stamp()

    # ------------------------------------------------------------------
    def _age_sec(self, state: SignalState, now: Time) -> Optional[float]:
        if state.last_seen is None:
            return None
        return (now - state.last_seen).nanoseconds / 1e9

    def _base_status(self, name: str, hw: str) -> DiagnosticStatus:
        s = DiagnosticStatus()
        s.name = name
        s.hardware_id = hw
        return s

    # ------------------------------------------------------------------
    def _publish_diag(self):
        now = self._stamp()
        arr = DiagnosticArray()
        arr.header.stamp = now.to_msg()

        arr.status = [
            self._motor_status(now),
            self._imu_status(now),
            self._encoder_status(now),
            self._camera_status(now),
            self._network_status(now),
        ]

        self.pub.publish(arr)

    # ---------------- Individual Status Builders ----------------
    def _motor_status(self, now: Time):
        s = self._base_status("motor_temperature", "motors")
        age = self._age_sec(self.motor_state, now)

        if age is None or age > self.timeout_sec:
            s.level = DiagnosticStatus.WARN
            s.message = "No data"
            return s

        temp = self.motor_state.last_value or 0.0
        warn = self.get_parameter("motor_temp_warn").value
        err = self.get_parameter("motor_temp_error").value

        s.values.append(KeyValue("temperature_c", f"{temp:.2f}"))
        s.values.append(KeyValue("age_sec", f"{age:.2f}"))

        if temp >= err:
            s.level = DiagnosticStatus.ERROR
            s.message = "Overheat"
        elif temp >= warn:
            s.level = DiagnosticStatus.WARN
            s.message = "High temperature"
        else:
            s.level = DiagnosticStatus.OK
            s.message = "OK"
        return s

    def _imu_status(self, now: Time):
        s = self._base_status("imu_errors", "imu")
        age = self._age_sec(self.imu_state, now)

        if age is None or age > self.timeout_sec:
            s.level = DiagnosticStatus.WARN
            s.message = "No data"
        elif self.imu_state.last_text:
            s.level = DiagnosticStatus.ERROR
            s.message = self.imu_state.last_text
        else:
            s.level = DiagnosticStatus.OK
            s.message = "OK"
        return s

    def _encoder_status(self, now: Time):
        s = self._base_status("encoder_errors", "encoders")
        age = self._age_sec(self.encoder_state, now)

        if age is None or age > self.timeout_sec:
            s.level = DiagnosticStatus.WARN
            s.message = "No data"
        elif self.encoder_state.last_text:
            s.level = DiagnosticStatus.ERROR
            s.message = self.encoder_state.last_text
        else:
            s.level = DiagnosticStatus.OK
            s.message = "OK"
        return s

    def _camera_status(self, now: Time):
        s = self._base_status("camera_status", "camera")
        age = self._age_sec(self.camera_state, now)

        if age is None or age > self.timeout_sec:
            s.level = DiagnosticStatus.WARN
            s.message = "No data"
            return s

        text = (self.camera_state.last_text or "").lower()
        if text in ("ok", "ready", "true"):
            s.level = DiagnosticStatus.OK
            s.message = "OK"
        else:
            s.level = DiagnosticStatus.WARN
            s.message = text or "Unknown"
        return s

    def _network_status(self, now: Time):
        s = self._base_status("network_quality", "network")
        age = self._age_sec(self.network_state, now)

        if age is None or age > self.timeout_sec:
            s.level = DiagnosticStatus.WARN
            s.message = "No data"
            return s

        q = self.network_state.last_value or 0.0
        warn = self.get_parameter("network_warn").value
        err = self.get_parameter("network_error").value

        s.values.append(KeyValue("quality_percent", f"{q:.1f}"))
        s.values.append(KeyValue("age_sec", f"{age:.2f}"))

        if q <= err:
            s.level = DiagnosticStatus.ERROR
            s.message = "Poor"
        elif q <= warn:
            s.level = DiagnosticStatus.WARN
            s.message = "Weak"
        else:
            s.level = DiagnosticStatus.OK
            s.message = "OK"
        return s


def main(args=None):
    rclpy.init(args=args)
    node = RobotDiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
