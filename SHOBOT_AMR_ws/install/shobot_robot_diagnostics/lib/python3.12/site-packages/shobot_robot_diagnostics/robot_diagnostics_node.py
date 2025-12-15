#!/usr/bin/env python3
"""
Robot Diagnostics Node.

Publishes diagnostic_msgs/DiagnosticArray on /diagnostics summarizing:
- Motor temperature (Float32)
- IMU errors (String)
- Encoder errors (String)
- Camera status (String/Bool)
- Network quality (Float32 0-100)

Each input is monitored with a timeout; stale data is marked as WARN. Thresholds
are configurable via parameters.
"""

from dataclasses import dataclass
from typing import Callable, Optional

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

        # Parameters
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("timeout_sec", 2.0)
        self.declare_parameter("motor_temp_topic", "/motor/temperature")
        self.declare_parameter("motor_temp_warn", 70.0)
        self.declare_parameter("motor_temp_error", 85.0)
        self.declare_parameter("imu_error_topic", "/imu/errors")
        self.declare_parameter("encoder_error_topic", "/encoders/errors")
        self.declare_parameter("camera_status_topic", "/camera/status")
        self.declare_parameter("network_quality_topic", "/network/quality")
        self.declare_parameter("network_warn", 40.0)
        self.declare_parameter("network_error", 15.0)

        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        # State containers
        self.motor_state = SignalState()
        self.imu_state = SignalState()
        self.encoder_state = SignalState()
        self.camera_state = SignalState()
        self.network_state = SignalState()

        # Subscriptions
        self.create_subscription(
            Float32,
            self.get_parameter("motor_temp_topic").value,
            self._wrap_cb(self._motor_cb),
            qos,
        )
        self.create_subscription(
            String,
            self.get_parameter("imu_error_topic").value,
            self._wrap_cb(self._imu_cb),
            qos,
        )
        self.create_subscription(
            String,
            self.get_parameter("encoder_error_topic").value,
            self._wrap_cb(self._encoder_cb),
            qos,
        )
        self.create_subscription(
            String,
            self.get_parameter("camera_status_topic").value,
            self._wrap_cb(self._camera_cb),
            qos,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("camera_status_topic").value,
            self._wrap_cb(self._camera_bool_cb),
            qos,
        )
        self.create_subscription(
            Float32,
            self.get_parameter("network_quality_topic").value,
            self._wrap_cb(self._network_cb),
            qos,
        )

        self.pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(1.0 / max(self.publish_rate, 0.1), self._publish_diag)

        self.get_logger().info(
            "Robot diagnostics ready (publishing /diagnostics)."
        )

    # ------------------------------------------------------------------ #
    def _wrap_cb(self, fn: Callable):
        def _cb(msg):
            try:
                fn(msg)
            except Exception as exc:  # pragma: no cover - defensive
                self.get_logger().warn(f"Diagnostics callback error: {exc}")
        return _cb

    # Individual callbacks update state
    def _motor_cb(self, msg: Float32):
        self.motor_state.last_value = float(msg.data)
        self.motor_state.last_seen = self.get_clock().now()

    def _imu_cb(self, msg: String):
        self.imu_state.last_text = msg.data
        self.imu_state.last_seen = self.get_clock().now()

    def _encoder_cb(self, msg: String):
        self.encoder_state.last_text = msg.data
        self.encoder_state.last_seen = self.get_clock().now()

    def _camera_cb(self, msg: String):
        self.camera_state.last_text = msg.data
        self.camera_state.last_seen = self.get_clock().now()

    def _camera_bool_cb(self, msg: Bool):
        self.camera_state.last_text = "ok" if msg.data else "fault"
        self.camera_state.last_seen = self.get_clock().now()

    def _network_cb(self, msg: Float32):
        self.network_state.last_value = float(msg.data)
        self.network_state.last_seen = self.get_clock().now()

    # ------------------------------------------------------------------ #
    def _publish_diag(self):
        now = self.get_clock().now()
        msg = DiagnosticArray()
        msg.header.stamp = now.to_msg()

        msg.status = [
            self._motor_status(now),
            self._imu_status(now),
            self._encoder_status(now),
            self._camera_status(now),
            self._network_status(now),
        ]

        self.pub.publish(msg)

    def _staleness(self, state: SignalState, now: Time) -> Optional[float]:
        if state.last_seen is None:
            return None
        return (now - state.last_seen).nanoseconds / 1e9

    def _motor_status(self, now: Time) -> DiagnosticStatus:
        warn = float(self.get_parameter("motor_temp_warn").value)
        error = float(self.get_parameter("motor_temp_error").value)
        status = DiagnosticStatus(name="motor_temperature", hardware_id="motors")
        age = self._staleness(self.motor_state, now)
        if age is None or age > self.timeout_sec:
            status.level = DiagnosticStatus.WARN
            status.message = "No data"
        else:
            temp = self.motor_state.last_value or 0.0
            status.values.append(KeyValue(key="temperature_c", value=f"{temp:.2f}"))
            if temp >= error:
                status.level = DiagnosticStatus.ERROR
                status.message = "Overheat"
            elif temp >= warn:
                status.level = DiagnosticStatus.WARN
                status.message = "High temperature"
            else:
                status.level = DiagnosticStatus.OK
                status.message = "OK"
        return status

    def _imu_status(self, now: Time) -> DiagnosticStatus:
        status = DiagnosticStatus(name="imu_errors", hardware_id="imu")
        age = self._staleness(self.imu_state, now)
        if age is None or age > self.timeout_sec:
            status.level = DiagnosticStatus.WARN
            status.message = "No data"
        else:
            text = self.imu_state.last_text or ""
            if text.strip():
                status.level = DiagnosticStatus.ERROR
                status.message = text
            else:
                status.level = DiagnosticStatus.OK
                status.message = "OK"
        return status

    def _encoder_status(self, now: Time) -> DiagnosticStatus:
        status = DiagnosticStatus(name="encoder_errors", hardware_id="encoders")
        age = self._staleness(self.encoder_state, now)
        if age is None or age > self.timeout_sec:
            status.level = DiagnosticStatus.WARN
            status.message = "No data"
        else:
            text = self.encoder_state.last_text or ""
            if text.strip():
                status.level = DiagnosticStatus.ERROR
                status.message = text
            else:
                status.level = DiagnosticStatus.OK
                status.message = "OK"
        return status

    def _camera_status(self, now: Time) -> DiagnosticStatus:
        status = DiagnosticStatus(name="camera_status", hardware_id="camera")
        age = self._staleness(self.camera_state, now)
        if age is None or age > self.timeout_sec:
            status.level = DiagnosticStatus.WARN
            status.message = "No data"
        else:
            text = (self.camera_state.last_text or "").lower()
            if text in ("ok", "ready", "true"):
                status.level = DiagnosticStatus.OK
                status.message = "OK"
            elif text:
                status.level = DiagnosticStatus.WARN
                status.message = text
            else:
                status.level = DiagnosticStatus.OK
                status.message = "OK"
        return status

    def _network_status(self, now: Time) -> DiagnosticStatus:
        warn = float(self.get_parameter("network_warn").value)
        error = float(self.get_parameter("network_error").value)
        status = DiagnosticStatus(name="network_quality", hardware_id="network")
        age = self._staleness(self.network_state, now)
        if age is None or age > self.timeout_sec:
            status.level = DiagnosticStatus.WARN
            status.message = "No data"
        else:
            quality = self.network_state.last_value or 0.0
            status.values.append(KeyValue(key="quality_percent", value=f"{quality:.1f}"))
            if quality <= error:
                status.level = DiagnosticStatus.ERROR
                status.message = "Poor"
            elif quality <= warn:
                status.level = DiagnosticStatus.WARN
                status.message = "Weak"
            else:
                status.level = DiagnosticStatus.OK
                status.message = "OK"
        return status


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
