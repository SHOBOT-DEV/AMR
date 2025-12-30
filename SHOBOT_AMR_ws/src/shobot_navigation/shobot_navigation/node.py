#!/usr/bin/env python3
"""
Topic-driven Nav2 NavigateToPose client with optional path execution and cancel support.
"""

from collections import deque
from typing import Deque, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time

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

        goal_topic = self.get_parameter("goal_topic").value
        status_topic = self.get_parameter("status_topic").value
        feedback_topic = self.get_parameter("feedback_topic").value
        action_name = self.get_parameter("nav2_action_name").value
        self.planner_mode = self.get_parameter("planner_mode").value.lower()
        path_goal_topic = self.get_parameter("path_goal_topic").value
        self.feedback_throttle = float(self.get_parameter("feedback_throttle_sec").value)

        if self.planner_mode not in ("dwa", "path"):
            self.get_logger().warn(
                f"Invalid planner_mode '{self.planner_mode}', falling back to 'dwa'"
            )
            self.planner_mode = "dwa"

        # ---------------- Action client ----------------
        self.nav_client = ActionClient(self, NavigateToPose, action_name)

        # ---------------- Publishers / Subscribers / Services ----------------
        self.create_subscription(PoseStamped, goal_topic, self.goal_cb, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.feedback_pub = self.create_publisher(String, feedback_topic, 10)
        self.create_service(Trigger, "cancel_navigation", self.cancel_cb)

        self.path_sub = None
        self.path_queue: Deque[PoseStamped] = deque()
        self.current_goal_source: Optional[str] = None

        if self.planner_mode == "path":
            self.path_sub = self.create_subscription(
                Path, path_goal_topic, self.path_cb, 10
            )
            self.get_logger().info(
                f"Path mode enabled; listening on {path_goal_topic}"
            )

        # ---------------- State ----------------
        self.current_goal_handle = None
        self.last_feedback_time = self.get_clock().now()

        # ---------------- Timers ----------------
        self.wait_timer = self.create_timer(1.0, self._wait_for_server)

        self.get_logger().info(
            f"Navigation client started. Waiting for Nav2 action server '{action_name}'."
        )

    # ------------------------------------------------------------------
    def _wait_for_server(self):
        """Wait asynchronously for Nav2 action server."""
        if self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info("Nav2 action server available.")
            self.wait_timer.cancel()

    # ------------------------------------------------------------------
    def goal_cb(self, msg: PoseStamped):
        """Handle single Pose goal."""
        self._send_pose_goal(msg, source="single")

    # ------------------------------------------------------------------
    def path_cb(self, msg: Path):
        """Receive a Path and schedule it as sequential goals."""
        if not msg.poses:
            self.publish_status("Received empty path; ignoring.")
            return

        self.path_queue = deque(
            [PoseStamped(header=p.header, pose=p.pose) for p in msg.poses]
        )
        self.publish_status(f"Received path with {len(self.path_queue)} poses.")

        if self.current_goal_handle is None:
            self._send_next_path_goal()

    # ------------------------------------------------------------------
    def _send_next_path_goal(self):
        if not self.path_queue:
            self.publish_status("Path execution completed.")
            return
        self._send_pose_goal(self.path_queue.popleft(), source="path")

    # ------------------------------------------------------------------
    def _send_pose_goal(self, msg: PoseStamped, source: str):
        if not self.nav_client.server_is_ready():
            self.publish_status("Nav2 action server not ready; goal dropped.")
            return

        if self.current_goal_handle is not None:
            self.publish_status("Canceling previous goal before sending new one.")
            self._cancel_goal_internal(clear_path=False)

        goal = NavigateToPose.Goal()
        goal.pose = msg

        send_future = self.nav_client.send_goal_async(
            goal, feedback_callback=self.feedback_cb
        )
        send_future.add_done_callback(self._goal_response_cb)

        self.current_goal_source = source
        self.publish_status(
            f"Sending goal: frame={msg.header.frame_id}, "
            f"x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}"
        )

    # ------------------------------------------------------------------
    def _goal_response_cb(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.publish_status("Nav2 goal rejected.")
            self.current_goal_handle = None
            return

        self.current_goal_handle = goal_handle
        self.publish_status("Nav2 goal accepted.")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    # ------------------------------------------------------------------
    def feedback_cb(self, feedback_msg):
        now = self.get_clock().now()
        if (now - self.last_feedback_time).nanoseconds * 1e-9 < self.feedback_throttle:
            return

        self.last_feedback_time = now
        fb = feedback_msg.feedback

        if hasattr(fb, "distance_remaining"):
            text = f"distance_remaining={fb.distance_remaining:.2f}"
            if hasattr(fb, "speed"):
                text += f", speed={fb.speed:.2f}"
            self.feedback_pub.publish(String(data=text))

    # ------------------------------------------------------------------
    def _result_cb(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            msg = "Goal reached successfully."
        elif status == GoalStatus.STATUS_ABORTED:
            msg = "Navigation aborted."
        elif status == GoalStatus.STATUS_CANCELED:
            msg = "Navigation canceled."
        else:
            msg = f"Navigation finished with status {status}"

        self.publish_status(msg)
        self.current_goal_handle = None

        if (
            self.planner_mode == "path"
            and self.current_goal_source == "path"
            and status == GoalStatus.STATUS_SUCCEEDED
        ):
            self._send_next_path_goal()

    # ------------------------------------------------------------------
    def cancel_cb(self, request, response):
        response.success = self._cancel_goal_internal(clear_path=True)
        response.message = (
            "Cancel request sent." if response.success else "No active goal."
        )
        return response

    # ------------------------------------------------------------------
    def _cancel_goal_internal(self, clear_path: bool) -> bool:
        if self.current_goal_handle is None:
            return False

        try:
            self.current_goal_handle.cancel_goal_async()
            self.publish_status("Cancel request sent to Nav2.")
        except Exception as exc:
            self.get_logger().error(f"Cancel failed: {exc}")
            return False

        self.current_goal_handle = None
        if clear_path:
            self.path_queue.clear()
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
