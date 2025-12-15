#!/usr/bin/env python3
"""
Mission Control Node
=========================================================
Executes mission queues containing waypoint poses and
optional docking tasks. Integrates with Nav2 and docking
action servers. Fully compatible with SHOBOT architecture.
"""

import json
from collections import deque
from typing import Deque, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from std_srvs.srv import Trigger

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from shobot_docking.action import Dock


class MissionControl(Node):
    """Executes sequential missions: (Pose + optional Dock)."""

    def __init__(self):
        super().__init__("shobot_mission_control")

        # ---------------- Parameters ----------------
        self.declare_parameter("mission_topic", "/mission_queue")
        self.declare_parameter("task_topic", "/task_goal")
        self.declare_parameter("status_topic", "/mission_status")
        self.declare_parameter("nav2_action_name", "navigate_to_pose")
        self.declare_parameter("dock_action_name", "dock")

        mission_topic = self.get_parameter("mission_topic").value
        task_topic = self.get_parameter("task_topic").value
        status_topic = self.get_parameter("status_topic").value
        nav_action_name = self.get_parameter("nav2_action_name").value
        dock_action_name = self.get_parameter("dock_action_name").value

        # ---------------- Message interfaces ----------------
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.create_subscription(String, mission_topic, self.mission_cb, 10)
        self.create_subscription(PoseStamped, task_topic, self.task_cb, 10)
        self.cancel_srv = self.create_service(Trigger, "cancel_mission", self.cancel_cb)

        # ---------------- Mission queue ----------------
        self.missions: Deque[dict] = deque()
        self.executing = False

        # ---------------- Action Clients ----------------
        self.nav_client = ActionClient(self, NavigateToPose, nav_action_name)
        self.dock_client = ActionClient(self, Dock, dock_action_name)

        self.current_nav_handle = None
        self.current_dock_handle = None

        self.nav_ready = False
        self.dock_ready = False

        # Check for action servers
        self.timer = self.create_timer(0.5, self._ensure_servers)

        self.get_logger().info("MissionControl ready — waiting for mission queue input.")

    # ===================================================================
    # Action server availability check
    # ===================================================================
    def _ensure_servers(self):
        if not self.nav_ready:
            if self.nav_client.wait_for_server(timeout_sec=0.0):
                self.nav_ready = True
                self.publish_status("Nav2 server available.")

        if not self.dock_ready:
            if self.dock_client.wait_for_server(timeout_sec=0.0):
                self.dock_ready = True
                self.publish_status("Docking server available.")

        if self.nav_ready and self.dock_ready:
            self.timer.cancel()
            # Start mission executor
            self.create_timer(0.25, self._execute_loop)

    # ===================================================================
    # Mission input handlers
    # ===================================================================
    def mission_cb(self, msg: String):
        """Receives mission JSON: { "missions":[{pose:{...}, dock:bool}, ...]}"""
        try:
            data = json.loads(msg.data)
            missions = data.get("missions", [])
            if not isinstance(missions, list):
                raise ValueError("'missions' must be a list")
            self.missions.extend(missions)
            self.publish_status(f"Added {len(missions)} mission(s). Queue size: {len(self.missions)}")
        except Exception as e:
            self.publish_status(f"Invalid mission data: {e}")

    def task_cb(self, msg: PoseStamped):
        """Receives a single task pose (manual task insertion)."""
        mission = {
            "pose": self._pose_to_dict(msg),
            "dock": False
        }
        self.missions.append(mission)
        self.publish_status(f"Task added. Queue size: {len(self.missions)}")

    # ===================================================================
    # Mission execution loop
    # ===================================================================
    def _execute_loop(self):
        if self.executing:
            return
        if not self.missions:
            return
        if not (self.nav_ready and self.dock_ready):
            return

        mission = self.missions.popleft()
        self.executing = True

        self.publish_status(f"Executing mission: {json.dumps(mission)}")
        self._execute_mission(mission)

    def _execute_mission(self, mission: dict):
        pose_dict = mission.get("pose")
        do_dock = bool(mission.get("dock", False))

        # Validate pose
        if not pose_dict:
            self.publish_status("Mission missing 'pose'. Skipping.")
            self.executing = False
            return

        try:
            pose = self._pose_from_dict(pose_dict)
        except Exception as e:
            self.publish_status(f"Invalid pose format: {e}")
            self.executing = False
            return

        # Send Nav2 goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose

        future = self.nav_client.send_goal_async(nav_goal, feedback_callback=self._nav_feedback_cb)
        future.add_done_callback(lambda fut: self._nav_response_cb(fut, do_dock))

    # ===================================================================
    # Navigation action callbacks
    # ===================================================================
    def _nav_response_cb(self, future, do_dock: bool):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.publish_status("❌ Nav2 rejected the goal.")
            self.executing = False
            return

        self.publish_status("Nav2 accepted goal.")
        self.current_nav_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._nav_result_cb(fut, do_dock))

    def _nav_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.publish_status(
            f"Nav2 feedback → distance_remaining={fb.distance_remaining:.2f}, speed={fb.speed:.2f}"
        )

    def _nav_result_cb(self, future, do_dock: bool):
        result = future.result()
        status = result.status

        self.publish_status(f"Nav2 result: status={status}")

        self.current_nav_handle = None

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.publish_status("❌ Navigation failed.")
            self.executing = False
            return

        if do_dock:
            self._start_dock()
        else:
            self.publish_status("Mission complete.")
            self.executing = False

    # ===================================================================
    # Docking callbacks
    # ===================================================================
    def _start_dock(self):
        dock_goal = Dock.Goal()
        dock_goal.start = True

        send_future = self.dock_client.send_goal_async(dock_goal)
        send_future.add_done_callback(self._dock_response_cb)

    def _dock_response_cb(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.publish_status("❌ Docking goal rejected.")
            self.executing = False
            return

        self.publish_status("Docking goal accepted.")
        self.current_dock_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._dock_result_cb)

    def _dock_result_cb(self, future):
        result = future.result().result
        self.publish_status(f"Docking result: success={result.success}")

        self.current_dock_handle = None
        self.executing = False

    # ===================================================================
    # Cancellation
    # ===================================================================
    def cancel_cb(self, request, response):
        canceled = False

        if self.current_nav_handle:
            self.current_nav_handle.cancel_goal_async()
            canceled = True

        if self.current_dock_handle:
            self.current_dock_handle.cancel_goal_async()
            canceled = True

        if canceled:
            self.publish_status("Cancel request sent.")
            self.executing = False
        else:
            self.publish_status("No active mission to cancel.")

        response.success = canceled
        response.message = "Canceled" if canceled else "Nothing to cancel"
        return response

    # ===================================================================
    # Helpers
    # ===================================================================
    def publish_status(self, text: str):
        msg = String(data=text)
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    @staticmethod
    def _pose_to_dict(msg: PoseStamped) -> dict:
        return {
            "frame_id": msg.header.frame_id,
            "position": {
                "x": msg.pose.position.x,
                "y": msg.pose.position.y,
                "z": msg.pose.position.z,
            },
            "orientation": {
                "x": msg.pose.orientation.x,
                "y": msg.pose.orientation.y,
                "z": msg.pose.orientation.z,
                "w": msg.pose.orientation.w,
            },
        }

    @staticmethod
    def _pose_from_dict(data: dict) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = data.get("frame_id", "map")
        pose.pose.position.x = float(data["position"]["x"])
        pose.pose.position.y = float(data["position"]["y"])
        pose.pose.position.z = float(data["position"].get("z", 0.0))

        ori = data.get("orientation", {})
        pose.pose.orientation.x = float(ori.get("x", 0.0))
        pose.pose.orientation.y = float(ori.get("y", 0.0))
        pose.pose.orientation.z = float(ori.get("z", 0.0))
        pose.pose.orientation.w = float(ori.get("w", 1.0))

        return pose


# ===================================================================
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
