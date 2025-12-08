#!/usr/bin/env python3
"""Mission control: simple waypoint queue executor using Nav2 and docking."""
import json
from collections import deque
from typing import Deque, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from shobot_docking.action import Dock


class MissionControl(Node):
    """Consumes a list of missions (poses + optional dock) and executes sequentially."""

    def __init__(self):
        super().__init__("shobot_mission_control")

        self.declare_parameter("mission_topic", "/mission_queue")
        self.declare_parameter("status_topic", "/mission_status")
        self.declare_parameter("nav2_action_name", "navigate_to_pose")
        self.declare_parameter("dock_action_name", "dock")

        mission_topic = self.get_parameter("mission_topic").value
        status_topic = self.get_parameter("status_topic").value
        nav_action_name = self.get_parameter("nav2_action_name").value
        dock_action_name = self.get_parameter("dock_action_name").value

        self.missions: Deque[dict] = deque()
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.create_subscription(String, mission_topic, self.mission_cb, 10)

        self.nav_client = ActionClient(self, NavigateToPose, nav_action_name)
        self.dock_client = ActionClient(self, Dock, dock_action_name)
        self.cancel_srv = self.create_service(Trigger, "cancel_mission", self.cancel_cb)

        self.current_nav_handle = None
        self.current_dock_handle = None
        self.executing = False

        self.get_logger().info("shobot_mission_control ready for mission queue input.")
        self.timer = self.create_timer(0.5, self._ensure_servers)
        self.nav_ready = False
        self.dock_ready = False

    def _ensure_servers(self):
        if not self.nav_ready and self.nav_client.wait_for_server(timeout_sec=0.0):
            self.nav_ready = True
            self.publish_status("Nav2 action server available.")
        if not self.dock_ready and self.dock_client.wait_for_server(timeout_sec=0.0):
            self.dock_ready = True
            self.publish_status("Dock action server available.")
        if self.nav_ready and self.dock_ready:
            self.timer.cancel()
            # Start execution loop if missions queued
            self.create_timer(0.2, self._execute_loop)

    def mission_cb(self, msg: String):
        """Expect JSON: {"missions":[{"pose":{...},"dock":true/false}, ...]}"""
        try:
            data = json.loads(msg.data)
            missions = data.get("missions", [])
            if not isinstance(missions, list):
                raise ValueError("missions must be a list")
            self.missions.extend(missions)
            self.publish_status(f"Queued {len(missions)} missions. Total queue: {len(self.missions)}")
        except Exception as e:
            self.publish_status(f"Failed to parse mission: {e}")

    def _execute_loop(self):
        if self.executing or not self.missions or not (self.nav_ready and self.dock_ready):
            return
        mission = self.missions.popleft()
        self.executing = True
        self.publish_status(f"Starting mission: {mission}")
        self._execute_mission(mission)

    def _execute_mission(self, mission: dict):
        pose_dict = mission.get("pose")
        do_dock = bool(mission.get("dock", False))
        if not pose_dict:
            self.publish_status("Mission missing pose; skipping.")
            self.executing = False
            return

        try:
            pose = self._pose_from_dict(pose_dict)
        except Exception as e:
            self.publish_status(f"Invalid pose in mission: {e}")
            self.executing = False
            return

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose
        send_future = self.nav_client.send_goal_async(nav_goal, feedback_callback=self._nav_feedback_cb)
        send_future.add_done_callback(lambda fut: self._nav_response_cb(fut, do_dock))

    def _nav_response_cb(self, future, do_dock: bool):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.publish_status("Nav2 goal rejected.")
            self.executing = False
            return
        self.publish_status("Nav2 goal accepted.")
        self.current_nav_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._nav_result_cb(fut, do_dock))

    def _nav_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.publish_status(f"Nav2 feedback: dist_rem={fb.distance_remaining:.2f}, speed={fb.speed:.2f}")

    def _nav_result_cb(self, future, do_dock: bool):
        status = future.result().status
        self.publish_status(f"Nav2 result status: {status}")
        self.current_nav_handle = None
        if status != 0:  # STATUS_SUCCEEDED == 0
            self.executing = False
            return
        if do_dock:
            self._start_dock()
        else:
            self.executing = False

    def _start_dock(self):
        dock_goal = Dock.Goal()
        dock_goal.start = True
        send_future = self.dock_client.send_goal_async(dock_goal)
        send_future.add_done_callback(self._dock_response_cb)

    def _dock_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.publish_status("Dock goal rejected.")
            self.executing = False
            return
        self.publish_status("Dock goal accepted.")
        self.current_dock_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._dock_result_cb)

    def _dock_result_cb(self, future):
        res = future.result().result
        self.publish_status(f"Dock result: success={res.success}")
        self.current_dock_handle = None
        self.executing = False

    def cancel_cb(self, request, response):
        canceled = False
        if self.current_nav_handle:
            self.current_nav_handle.cancel_goal_async()
            canceled = True
        if self.current_dock_handle:
            self.current_dock_handle.cancel_goal_async()
            canceled = True
        if canceled:
            self.publish_status("Cancel requested for active mission.")
        else:
            self.publish_status("No active mission to cancel.")
        response.success = canceled
        response.message = "Cancel issued" if canceled else "Nothing to cancel"
        self.executing = False if canceled else self.executing
        return response

    def publish_status(self, text: str):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    @staticmethod
    def _pose_from_dict(data: dict) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = data.get("frame_id", "map")
        pose.pose.position.x = float(data["position"]["x"])
        pose.pose.position.y = float(data["position"]["y"])
        pose.pose.position.z = float(data["position"].get("z", 0.0))
        orientation = data.get("orientation", {})
        pose.pose.orientation.x = float(orientation.get("x", 0.0))
        pose.pose.orientation.y = float(orientation.get("y", 0.0))
        pose.pose.orientation.z = float(orientation.get("z", 0.0))
        pose.pose.orientation.w = float(orientation.get("w", 1.0))
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = MissionControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
