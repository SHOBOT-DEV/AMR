#!/usr/bin/env python3
"""Navigation service node to accept simple pose requests and forward to Nav2."""
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from shobot_navigation_server.srv import Navigate  # custom srv: string id + pose
from std_msgs.msg import String


class NavigationServer(Node):
    """Expose a service to accept poses and forward them to Nav2 with status."""

    def __init__(self):
        super().__init__("shobot_navigation_server")

        self.declare_parameter("nav2_action_name", "navigate_to_pose")
        self.declare_parameter("status_topic", "/navigation_server/status")
        action_name = self.get_parameter("nav2_action_name").value
        status_topic = self.get_parameter("status_topic").value

        self.nav_client = ActionClient(self, NavigateToPose, action_name)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.create_service(Navigate, "shobot_navigation_server/navigate", self.handle_nav_request)

        self.current_goal_handle = None
        self.get_logger().info(f"Navigation server ready. Waiting on Nav2 action '{action_name}'.")

    def handle_nav_request(self, request, response):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            response.success = False
            response.message = "Nav2 action server not available"
            self.publish_status(response.message)
            return response

        # Cancel any existing goal
        if self.current_goal_handle is not None:
            self._cancel_current_goal()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = request.pose

        future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
        future.add_done_callback(self._goal_response_cb)

        response.success = True
        response.message = f"Goal accepted for request {request.request_id}"
        self.publish_status(response.message)
        return response

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

    def _result_cb(self, future):
        status = future.result().status
        self.publish_status(f"Nav2 goal completed with status: {status}")
        self.current_goal_handle = None

    def _cancel_current_goal(self):
        self.publish_status("Cancelling current Nav2 goal.")
        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(lambda _: self.publish_status("Cancel request sent."))
        self.current_goal_handle = None

    def feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.publish_status(
            f"Feedback: distance_remaining={feedback.distance_remaining:.2f}, speed={feedback.speed:.2f}"
        )

    def publish_status(self, text: str):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
