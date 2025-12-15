#!/usr/bin/env python3
"""
Navigation service node for SHOBOT AMR.
Accepts simple navigation service requests and forwards them to Nav2.
Publishes status updates continuously.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from shobot_navigation_server.srv import Navigate


class NavigationServer(Node):
    """Expose a simple service interface to control Nav2 navigation."""

    def __init__(self):
        super().__init__("shobot_navigation_server")

        # ---------------- Parameters ----------------
        self.declare_parameter("nav2_action_name", "/navigate_to_pose")
        self.declare_parameter("status_topic", "/navigation_server/status")
        self.declare_parameter("feedback_throttle_sec", 0.5)
        self.declare_parameter("goal_timeout_sec", 0.0)   # 0 = disabled

        action_name = self.get_parameter("nav2_action_name").value
        status_topic = self.get_parameter("status_topic").value
        self.feedback_throttle = float(self.get_parameter("feedback_throttle_sec").value)
        self.goal_timeout = float(self.get_parameter("goal_timeout_sec").value)

        # ---------------- Nav2 Action Client ----------------
        self.nav_client = ActionClient(self, NavigateToPose, action_name)
        self.current_goal_handle = None
        self.last_feedback_time = self.get_clock().now()

        # ---------------- Publishers & Services ----------------
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.create_service(
            Navigate,
            "shobot_navigation_server/navigate",
            self.handle_nav_request
        )

        self.get_logger().info(
            f"[NavigationServer] Waiting for Nav2 action server: {action_name}"
        )

        # Wait asynchronously for Nav2
        self.nav_client.wait_for_server()

        self.get_logger().info("[NavigationServer] Ready.")

    # ----------------------------------------------------------------------
    def handle_nav_request(self, request, response):
        """Receive a service request and send a Nav2 goal."""

        # Cancel existing goal
        if self.current_goal_handle is not None:
            self.cancel_current_goal()

        # Build navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = request.pose

        # Send goal to Nav2
        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_cb
        )
        future.add_done_callback(self.goal_response_cb)

        response.success = True
        response.message = f"Goal submitted for request ID: {request.request_id}"
        self.publish_status(response.message)
        return response

    # ----------------------------------------------------------------------
    def goal_response_cb(self, future):
        """Called when Nav2 responds to the goal request."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.current_goal_handle = None
            self.publish_status("Goal rejected by Nav2.")
            return

        self.current_goal_handle = goal_handle
        self.publish_status("Goal accepted by Nav2.")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    # ----------------------------------------------------------------------
    def result_cb(self, future):
        """Handle Nav2 navigation result."""
        result = future.result()
        status = result.status

        self.publish_status(f"Navigation completed with status: {status}")
        self.current_goal_handle = None

    # ----------------------------------------------------------------------
    def cancel_current_goal(self):
        """Cancel an active Nav2 goal."""
        if self.current_goal_handle is None:
            return

        self.publish_status("Cancelling active Nav2 goal...")
        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(lambda _: self.publish_status("Goal cancellation request sent."))
        self.current_goal_handle = None

    # ----------------------------------------------------------------------
    def feedback_cb(self, feedback_msg):
        """Handle continuous feedback from Nav2 (distance remaining, speed)."""

        now = self.get_clock().now()
        if (now - self.last_feedback_time).nanoseconds * 1e-9 < self.feedback_throttle:
            return  # throttle feedback

        self.last_feedback_time = now
        fb = feedback_msg.feedback

        self.publish_status(
            f"Feedback: distance_remaining={fb.distance_remaining:.2f}  speed={fb.speed:.2f}"
        )

    # ----------------------------------------------------------------------
    def publish_status(self, text: str):
        """Publish status messages to topic + log."""
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
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
