#!/usr/bin/env python3
import json
from threading import Thread
from typing import List, Optional

import rclpy
from flask import Flask, jsonify, request
from flask_cors import CORS
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

app = Flask("shobot_api_bridge")
CORS(app)


class ApiBridge(Node):
    def __init__(self):
        super().__init__("shobot_api_bridge")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("status_topic", "/status_text")

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        status_topic = self.get_parameter("status_topic").value

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.status_sub = self.create_subscription(
            Twist, status_topic, self.status_callback, 10
        )
        self.latest_status = {}
        self.get_logger().info(
            f"Bridge running (cmd_vel: {cmd_vel_topic}, status: {status_topic})"
        )

    def status_callback(self, msg):
        # Store latest status; extend with real fields when available.
        self.latest_status = {
            "vx": msg.linear.x,
            "vy": msg.linear.y,
            "wz": msg.angular.z,
        }

    def publish_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.linear.y = float(linear_y)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)


node: Optional[ApiBridge] = None
executor: Optional[MultiThreadedExecutor] = None
spin_thread: Optional[Thread] = None


def start_ros():
    global node, executor, spin_thread
    rclpy.init()
    node = ApiBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    def spin():
        executor.spin()

    spin_thread = Thread(target=spin, daemon=True)
    spin_thread.start()


def shutdown_ros():
    global node, executor
    if executor is not None:
        executor.shutdown()
    if node is not None:
        node.destroy_node()
    rclpy.shutdown()


@app.route("/api/status", methods=["GET"])
def get_status():
    if node is None:
        return jsonify({"status": "offline"}), 503
    return jsonify({"status": "online", "latest_status": node.latest_status})


@app.route("/api/cmd_vel", methods=["POST"])
def set_cmd_vel():
    if node is None:
        return jsonify({"success": False, "error": "ROS2 not started"}), 503

    payload = request.get_json(silent=True) or {}
    linear_x = payload.get("linear_x", request.args.get("linear_x", 0.0))
    linear_y = payload.get("linear_y", request.args.get("linear_y", 0.0))
    angular_z = payload.get("angular_z", request.args.get("angular_z", 0.0))

    try:
        node.publish_cmd_vel(float(linear_x), float(linear_y), float(angular_z))
    except Exception as exc:  # noqa: BLE001
        return jsonify({"success": False, "error": str(exc)}), 500

    return jsonify({"success": True})


def main():
    start_ros()
    try:
        app.run(host="0.0.0.0", port=8000, threaded=True)
    finally:
        shutdown_ros()


if __name__ == "__main__":
    main()
