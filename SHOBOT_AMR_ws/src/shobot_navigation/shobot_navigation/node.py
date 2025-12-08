#!/usr/bin/env python3
"""Simple Nav2 NavigateToPose client with topic-driven goals and cancel support."""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from std_srvs.srv import Trigger


class NavigationClient(Node):
    """Subscribe for PoseStamped goals, forward to Nav2, and publish status/feedback."""

    def __init__(self):
        super().__init__("shobot_navigation")

        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("status_topic", "/navigation_status")
        self.declare_parameter("feedback_topic", "/navigation_feedback")
        self.declare_parameter("nav2_action_name", "navigate_to_pose")

        goal_topic = self.get_parameter("goal_topic").value
        status_topic = self.get_parameter("status_topic").value
        feedback_topic = self.get_parameter("feedback_topic").value
        action_name = self.get_parameter("nav2_action_name").value

        self.nav_client = ActionClient(self, NavigateToPose, action_name)

        self.goal_sub = self.create_subscription(PoseStamped, goal_topic, self.goal_cb, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.feedback_pub = self.create_publisher(String, feedback_topic, 10)
        self.cancel_srv = self.create_service(Trigger, "cancel_navigation", self.cancel_cb)

        self.current_goal_handle = None
        self.get_logger().info(
            f"Waiting for Nav2 action server '{action_name}' and listening for goals on {goal_topic}"
        )

        self.timer = self.create_timer(0.5, self._spin_until_server)
        self.server_ready = False

    def _spin_until_server(self):
        if self.server_ready:
            return
        if self.nav_client.wait_for_server(timeout_sec=0.0):
            self.server_ready = True
            self.get_logger().info("Nav2 action server available.")
            self.timer.cancel()

    def goal_cb(self, msg: PoseStamped):
        if not self.server_ready and not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Nav2 action server not available; cannot send goal yet.")
            return

        if self.current_goal_handle is not None:
            self.get_logger().info("Cancelling previous goal before sending a new one.")
            self._cancel_goal()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg

        send_future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)
        self.publish_status(f"Sending goal to Nav2: {msg.header.frame_id} [{msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}]")

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

    def feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.feedback_pub.publish(
            String(data=f"distance_remaining: {feedback.distance_remaining:.2f}, speed: {feedback.speed:.2f}")
        )

    def _result_cb(self, future):
        status = future.result().status
        self.publish_status(f"Nav2 goal completed with status: {status}")
        self.current_goal_handle = None

    def cancel_cb(self, request, response):
        success = self._cancel_goal()
        response.success = success
        response.message = "Cancel requested" if success else "No active goal to cancel"
        return response

    def _cancel_goal(self):
        if self.current_goal_handle is None:
            return False
        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(lambda _: self.publish_status("Cancel request sent to Nav2."))
        self.current_goal_handle = None
        return True

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
