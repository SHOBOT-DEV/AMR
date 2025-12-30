#!/usr/bin/env python3
"""
Navigation service node for SHOBOT AMR.

- Exposes a simple service API to send navigation goals
- Internally forwards goals to Nav2 NavigateToPose action
- Publishes continuous navigation status updates
- Supports goal cancellation and optional timeout
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import String

from shobot_navigation_server.srv import Navigate


class NavigationServer(Node):
    """Simple service wrapper around Nav2 navigation."""

    def __init__(self):
        super().__init__("shobot_navigation_server")

        # ---------------- Parameters ----------------
        self.declare_parameter("nav2_action_name", "/navigate_to_pose")
        self.declare_parameter("status_topic", "/navigation_server/status")
        self.declare_parameter("feedback_throttle_sec", 0.5)
        self.declare_parameter("goal_timeout_sec", 0.0)  # 0 = disabled

        self.action_name = self.get_parameter("nav2_action_name").value
        self.feedback_throttle = float(
            self.get_parameter("feedback_throttle_sec").value
        )
        self.goal_timeout = float(
            self.get_parameter("goal_timeout_sec").value
        )

        status_topic = self.get_parameter("status_topic").value

        # ---------------- Nav2 Action Client ----------------
        self.nav_client = ActionClient(
            self, NavigateToPose, self.action_name
        )

        self.current_goal_handle = None
        self.goal_start_time: Time | None = None
        self.last_feedback_time = self.get_clock().now()

        # ---------------- Publishers & Services ----------------
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.create_service(
            Navigate,
            "shobot_navigation_server/navigate",
            self.handle_nav_request,
        )

        # ---------------- Timers ----------------
        self.wait_timer = self.create_timer(1.0, self._wait_for_nav2)
        self.timeout_timer = None

        self.get_logger().info(
            f"[NavigationServer] Waiting for Nav2 action server '{self.action_name}'"
        )

    # ------------------------------------------------------------------
    def _wait_for_nav2(self):
        """Wait asynchronously for Nav2 action server."""
        if self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info("[NavigationServer] Nav2 action server available.")
            self.wait_timer.cancel()

    # ------------------------------------------------------------------
    def handle_nav_request(self, request, response):
        """Handle incoming navigation service request."""

        if not self.nav_client.server_is_ready():
            response.success = False
            response.message = "Nav2 action server not available."
            return response

        # Cancel any active goal
        self.cancel_current_goal()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = request.pose

        send_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_cb,
        )
        send_future.add_done_callback(self.goal_response_cb)

        response.success = True
        response.message = f"Navigation goal submitted (id={request.request_id})"
        self.publish_status(response.message)
        return response

    # ------------------------------------------------------------------
    def goal_response_cb(self, future):
        """Handle Nav2 goal acceptance."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.current_goal_handle = None
            self.publish_status("Goal rejected by Nav2.")
            return

        self.current_goal_handle = goal_handle
        self.goal_start_time = self.get_clock().now()
        self.publish_status("Goal accepted by Nav2.")

        # Optional timeout
        if self.goal_timeout > 0.0:
            self.timeout_timer = self.create_timer(
                self.goal_timeout, self._goal_timeout_cb
            )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    # ------------------------------------------------------------------
    def result_cb(self, future):
        """Handle navigation result."""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            msg = "Navigation succeeded"
        elif status == GoalStatus.STATUS_ABORTED:
            msg = "Navigation aborted"
        elif status == GoalStatus.STATUS_CANCELED:
            msg = "Navigation canceled"
        else:
            msg = f"Navigation ended with status {status}"

        self.publish_status(msg)
        self._clear_goal_state()

    # ------------------------------------------------------------------
    def feedback_cb(self, feedback_msg):
        """Handle throttled feedback from Nav2."""
        now = self.get_clock().now()

        if (
            (now - self.last_feedback_time).nanoseconds * 1e-9
            < self.feedback_throttle
        ):
            return

        self.last_feedback_time = now
        fb = feedback_msg.feedback

        self.publish_status(
            f"Feedback: distance_remaining={fb.distance_remaining:.2f} "
            f"speed={fb.speed:.2f}"
        )

    # ------------------------------------------------------------------
    def cancel_current_goal(self):
        """Cancel active Nav2 goal if present."""
        if self.current_goal_handle is None:
            return

        self.publish_status("Canceling active navigation goal...")
        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            lambda _: self.publish_status("Cancel request sent.")
        )

        self._clear_goal_state()

    # ------------------------------------------------------------------
    def _goal_timeout_cb(self):
        """Cancel goal if timeout exceeded."""
        if self.current_goal_handle is None or self.goal_start_time is None:
            return

        self.publish_status("Navigation timeout exceeded. Canceling goal.")
        self.cancel_current_goal()

    # ------------------------------------------------------------------
    def _clear_goal_state(self):
        self.current_goal_handle = None
        self.goal_start_time = None

        if self.timeout_timer is not None:
            self.timeout_timer.cancel()
            self.timeout_timer = None

    # ------------------------------------------------------------------
    def publish_status(self, text: str):
        """Publish navigation status."""
        self.status_pub.publish(String(data=text))
        self.get_logger().info(f"[NavStatus] {text}")


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = NavigationServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
