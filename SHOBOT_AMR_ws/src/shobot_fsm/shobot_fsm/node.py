#!/usr/bin/env python3
"""
Finite State Machine (FSM) Node
Centralizes robot state transitions to keep behavior predictable.
States: IDLE, MOVING, BLOCKED, DOCKING, CHARGING, ERROR, RECOVERY.
"""

import json
from typing import Dict, Set

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Valid robot states
STATES = {
    "IDLE",
    "MOVING",
    "BLOCKED",
    "DOCKING",
    "CHARGING",
    "ERROR",
    "RECOVERY",
}

# Allowed transitions map: current -> permitted next states
ALLOWED_TRANSITIONS: Dict[str, Set[str]] = {
    "IDLE": {"MOVING", "DOCKING", "CHARGING", "ERROR"},
    "MOVING": {"IDLE", "BLOCKED", "DOCKING", "ERROR"},
    "BLOCKED": {"RECOVERY", "IDLE", "ERROR"},
    "RECOVERY": {"MOVING", "IDLE", "ERROR"},
    "DOCKING": {"CHARGING", "IDLE", "ERROR"},
    "CHARGING": {"IDLE", "ERROR"},
    "ERROR": {"RECOVERY", "IDLE"},
}


class RobotStateMachine(Node):
    """Manage robot state transitions and publish the current state."""

    def __init__(self):
        super().__init__("shobot_fsm")

        # ---------------- Parameters ----------------
        self.declare_parameter("state_topic", "/robot_state")
        self.declare_parameter("event_topic", "/robot_state_request")
        self.declare_parameter("status_topic", "/fsm_status")
        self.declare_parameter("heartbeat_hz", 1.0)
        self.declare_parameter("initial_state", "IDLE")

        self.state_topic = self.get_parameter("state_topic").value
        self.event_topic = self.get_parameter("event_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.heartbeat_hz = float(self.get_parameter("heartbeat_hz").value)

        initial_state = str(self.get_parameter("initial_state").value).upper()
        if initial_state not in STATES:
            self.get_logger().warn(
                f"Invalid initial_state '{initial_state}', defaulting to IDLE."
            )
            initial_state = "IDLE"

        self.state = initial_state

        # ---------------- Publishers / Subscribers ----------------
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(String, self.event_topic, self.event_cb, 10)

        # Heartbeat timer publishes the current state at a steady rate
        self.create_timer(1.0 / max(self.heartbeat_hz, 0.1), self._heartbeat)

        # Publish initial state immediately
        self._publish_state(reason="initial_state")
        self.publish_status(
            f"FSM ready. Listening for requests on {self.event_topic}."
        )

    # ------------------------------------------------------------------
    def event_cb(self, msg: String):
        """Handle incoming state change requests."""
        payload = msg.data.strip()
        if not payload:
            return

        target_state = None
        reason = ""
        force = False

        try:
            data = json.loads(payload)
            target_state = data.get("state") or data.get("target_state") or data.get("next")
            if target_state:
                target_state = str(target_state).upper()
            reason = data.get("reason", "")
            force = bool(data.get("force", False))
        except Exception:
            # Treat raw strings as direct state names
            target_state = payload.upper()
            reason = "string request"

        if target_state not in STATES:
            self.publish_status(f"Rejected state request '{target_state}': not a valid state.")
            return

        self._transition(target_state, reason=reason or "external request", force=force)

    # ------------------------------------------------------------------
    def _transition(self, target_state: str, reason: str = "", force: bool = False):
        """Attempt a state transition if allowed."""
        if target_state == self.state:
            self.publish_status(f"Already in state {self.state}.")
            return True

        allowed = ALLOWED_TRANSITIONS.get(self.state, set())
        if not force and target_state not in allowed:
            self.publish_status(
                f"Blocked transition {self.state} -> {target_state}. "
                f"Allowed: {sorted(allowed)}. Use force=true to override."
            )
            return False

        previous = self.state
        self.state = target_state
        self._publish_state(previous=previous, reason=reason, forced=force)
        return True

    # ------------------------------------------------------------------
    def _publish_state(self, previous: str = None, reason: str = "", forced: bool = False):
        """Publish the current state as JSON plus a human-readable status string."""
        stamp = self.get_clock().now().nanoseconds
        payload = {
            "state": self.state,
            "previous": previous,
            "reason": reason,
            "forced": forced,
            "timestamp": stamp,
        }

        self.state_pub.publish(String(data=json.dumps(payload)))
        self.publish_status(
            f"State {previous or 'None'} -> {self.state} (forced={forced}) reason='{reason}'"
        )

    # ------------------------------------------------------------------
    def _heartbeat(self):
        """Regularly republish the current state so late subscribers catch up."""
        self.state_pub.publish(
            String(
                data=json.dumps(
                    {
                        "state": self.state,
                        "previous": None,
                        "reason": "heartbeat",
                        "forced": False,
                        "timestamp": self.get_clock().now().nanoseconds,
                    }
                )
            )
        )

    # ------------------------------------------------------------------
    def publish_status(self, text: str):
        """Publish a status string for dashboards and logs."""
        msg = String(data=text)
        self.status_pub.publish(msg)
        self.get_logger().info(f"[FSM] {text}")


def main(args=None):
    rclpy.init(args=args)
    node = RobotStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
