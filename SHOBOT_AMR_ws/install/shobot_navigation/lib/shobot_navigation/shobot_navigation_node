#!/usr/bin/env python3
"""Simple Nav2 NavigateToPose client with topic-driven goals and cancel support."""
from collections import deque
from typing import Deque, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from std_srvs.srv import Trigger


class NavigationClient(Node):
    """Subscribe for PoseStamped goals, forward to Nav2, and publish status/feedback."""

    def __init__(self):
        super().__init__("shobot_navigation")

        # ---------------- Parameters ----------------
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("status_topic", "/navigation_status")
        self.declare_parameter("feedback_topic", "/navigation_feedback")
        self.declare_parameter("nav2_action_name", "/navigate_to_pose")
        self.declare_parameter("planner_mode", "dwa")  # dwa | path
        self.declare_parameter("path_goal_topic", "/plan_goal")
        self.declare_parameter("feedback_throttle_sec", 0.5)
        self.declare_parameter("server_wait_timeout", 2.0)

        goal_topic = self.get_parameter("goal_topic").value
        status_topic = self.get_parameter("status_topic").value
        feedback_topic = self.get_parameter("feedback_topic").value
        action_name = self.get_parameter("nav2_action_name").value
        self.planner_mode = self.get_parameter("planner_mode").value
        path_goal_topic = self.get_parameter("path_goal_topic").value
        self.feedback_throttle = float(self.get_parameter("feedback_throttle_sec").value)
        self.server_wait_timeout = float(self.get_parameter("server_wait_timeout").value)

        # ---------------- Action client ----------------
        self.nav_client = ActionClient(self, NavigateToPose, action_name)

        # ---------------- Publishers / Subscribers / Services ----------------
        self.goal_sub = self.create_subscription(PoseStamped, goal_topic, self.goal_cb, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.feedback_pub = self.create_publisher(String, feedback_topic, 10)
        self.cancel_srv = self.create_service(Trigger, "cancel_navigation", self.cancel_cb)

        self.path_sub = None
        self.path_queue: Deque[PoseStamped] = deque()
        self.current_goal_source = "single"

        if self.planner_mode == "path":
            self.path_sub = self.create_subscription(Path, path_goal_topic, self.path_cb, 10)
            self.get_logger().info(
                f"Planner mode 'path' enabled; listening for Path on {path_goal_topic}."
            )
        else:
            self.get_logger().info("Planner mode set to 'dwa' (Nav2 default).")

        self.current_goal_handle = None
        self.last_feedback_time = self.get_clock().now()

        self.get_logger().info(
            f"Navigation client initialized. Waiting for Nav2 action server '{action_name}' and listening for goals on {goal_topic}"
        )

        # Block until action server is available (with retries)
        waited = 0.0
        while not self.nav_client.wait_for_server(timeout_sec=self.server_wait_timeout):
            waited += self.server_wait_timeout
            self.get_logger().warn(f"Waiting for Nav2 action server... waited {waited:.1f}s")
        self.get_logger().info("Nav2 action server available.")

    # ------------------------------------------------------------------
    def goal_cb(self, msg: PoseStamped):
        """Handle a single Pose goal (used in both modes)."""
        self._send_pose_goal(msg, source="single")

    # ------------------------------------------------------------------
    def _send_pose_goal(self, msg: PoseStamped, source: str):
        """Send a pose as a NavigateToPose goal to Nav2."""
        if not self.nav_client.server_is_ready():
            self.get_logger().warn("Nav2 action server not ready; dropping goal.")
            return

        # Cancel any running goal before sending new one (single-goal behavior)
        if self.current_goal_handle is not None:
            self.get_logger().info("Cancelling previous goal before sending a new one.")
            self._cancel_goal()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg

        send_future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)
        self.current_goal_source = source

        self.publish_status(
            f"Sending goal to Nav2: frame={msg.header.frame_id} x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f}"
        )

    # ------------------------------------------------------------------
    def path_cb(self, msg: Path):
        """Receive a Path and schedule it as a sequence of pose goals."""
        if not msg.poses:
            self.get_logger().warn("Received empty path; ignoring.")
            return

        # store PoseStamped queue (make a copy to avoid mutating original)
        self.path_queue = deque([PoseStamped(p.header, p.pose) for p in msg.poses])
        self.publish_status(f"Received path with {len(self.path_queue)} poses; executing sequential goals.")
        # Start execution if no current goal
        if self.current_goal_handle is None:
            self._send_next_path_goal()

    # ------------------------------------------------------------------
    def _send_next_path_goal(self):
        """Send next pose from the path queue if available."""
        if not self.path_queue:
            self.publish_status("Path execution complete.")
            return
        next_pose = self.path_queue.popleft()
        self._send_pose_goal(next_pose, source="path")

    # ------------------------------------------------------------------
    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.publish_status("Nav2 goal rejected.")
            self.current_goal_handle = None
            return
        self.publish_status("Nav2 goal accepted.")
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    # ------------------------------------------------------------------
    def feedback_cb(self, feedback_msg):
        """Throttle and publish feedback info."""
        now = self.get_clock().now()
        dt = (now - self.last_feedback_time).nanoseconds / 1e9
        if dt < self.feedback_throttle:
            return
        self.last_feedback_time = now
        fb = feedback_msg.feedback
        try:
            self.feedback_pub.publish(
                String(data=f"distance_remaining: {fb.distance_remaining:.2f}, speed: {fb.speed:.2f}")
            )
        except Exception:
            # be defensive: some feedback implementations differ
            self.get_logger().debug("Unexpected feedback format; skipping publish.")

    # ------------------------------------------------------------------
    def _result_cb(self, future):
        result = future.result()
        status = result.status
        self.publish_status(f"Nav2 goal completed with status: {status}")
        # keep current_goal_handle until after we process follow-up
        self.current_goal_handle = None

        # If running path mode and goal succeeded, continue with next waypoint
        if (
            self.planner_mode == "path"
            and self.current_goal_source == "path"
            and status == GoalStatus.STATUS_SUCCEEDED
        ):
            self._send_next_path_goal()

    # ------------------------------------------------------------------
    def cancel_cb(self, request, response):
        """Service handler to cancel current navigation."""
        success = self._cancel_goal()
        response.success = success
        response.message = "Cancel requested" if success else "No active goal to cancel"
        return response

    # ------------------------------------------------------------------
    def _cancel_goal(self) -> bool:
        """Cancel active goal (if any)."""
        if self.current_goal_handle is None:
            return False

        try:
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda f: self.publish_status("Cancel request sent to Nav2."))
        except Exception as e:
            self.get_logger().error(f"Failed to send cancel request: {e}")
            return False

        # Clear local state and path queue
        self.current_goal_handle = None
        if self.planner_mode == "path":
            self.path_queue = deque()
            self.publish_status("Path queue cleared.")

        return True

    # ------------------------------------------------------------------
    def publish_status(self, text: str):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
