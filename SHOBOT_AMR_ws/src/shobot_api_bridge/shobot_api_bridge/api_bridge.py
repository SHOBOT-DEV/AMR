#!/usr/bin/env python3
"""
REST API Bridge for SHOBOT AMR
==============================
- Exposes ROS2 cmd_vel as HTTP endpoint
- Streams compressed camera frames via MJPEG
- Publishes and retrieves simple robot status
"""

import json
import time
from threading import Thread, Lock
from typing import Optional

import rclpy
from flask import Flask, jsonify, request, Response
from flask_cors import CORS
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

app = Flask("shobot_api_bridge")
CORS(app)


# ======================================================================
# ROS2 NODE
# ======================================================================

class ApiBridge(Node):
    """Bridge between ROS2 and HTTP API server."""

    def __init__(self):
        super().__init__("shobot_api_bridge")

        # ------------------- Parameters -------------------
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("status_topic", "/status_text")   # expects std_msgs/String
        self.declare_parameter("camera_topic", "/camera/image/compressed")
        self.declare_parameter("api_host", "0.0.0.0")
        self.declare_parameter("api_port", 8000)

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        status_topic = self.get_parameter("status_topic").value
        camera_topic = self.get_parameter("camera_topic").value

        # ------------------- Publishers -------------------
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # ------------------- Subscribers -------------------
        self.status_sub = self.create_subscription(
            String, status_topic, self.status_callback, 10
        )

        self.latest_status = {"text": "No status yet"}

        # Camera handling
        self.camera_lock = Lock()
        self.latest_camera: Optional[CompressedImage] = None

        self.create_subscription(
            CompressedImage, camera_topic, self.camera_callback, 10
        )

        self.get_logger().info(
            f"API Bridge started:\n"
            f"  cmd_vel → {cmd_vel_topic}\n"
            f"  status  ← {status_topic}\n"
            f"  camera  ← {camera_topic}"
        )

    # ------------------------------------------------------------------
    def status_callback(self, msg: String):
        """Store latest status text."""
        self.latest_status = {"text": msg.data}

    # ------------------------------------------------------------------
    def camera_callback(self, msg: CompressedImage):
        """Store latest JPEG frame."""
        with self.camera_lock:
            self.latest_camera = msg

    # ------------------------------------------------------------------
    def publish_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float):
        """Publish velocity commands."""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.linear.y = float(linear_y)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)


# ======================================================================
# GLOBAL ROS EXECUTION
# ======================================================================

node: Optional[ApiBridge] = None
executor: Optional[MultiThreadedExecutor] = None
spin_thread: Optional[Thread] = None
APP_HOST = "0.0.0.0"
APP_PORT = 8000


def start_ros(args=None):
    """Start ROS2 node + executor in background thread."""
    global node, executor, spin_thread, APP_HOST, APP_PORT

    rclpy.init(args=args)
    node = ApiBridge()
    APP_HOST = str(node.get_parameter("api_host").value)
    APP_PORT = int(node.get_parameter("api_port").value)

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    def spin():
        executor.spin()

    spin_thread = Thread(target=spin, daemon=True)
    spin_thread.start()


def shutdown_ros():
    """Shutdown ROS2 cleanly."""
    global node, executor

    if executor:
        executor.shutdown()

    if node:
        node.destroy_node()

    rclpy.shutdown()


# ======================================================================
# FLASK HTTP API
# ======================================================================

@app.route("/api/status", methods=["GET"])
def api_get_status():
    if node is None:
        return jsonify({"status": "offline"}), 503
    return jsonify({
        "status": "online",
        "latest_status": node.latest_status
    })


@app.route("/api/cmd_vel", methods=["POST"])
def api_cmd_vel():
    if node is None:
        return jsonify({"success": False, "error": "ROS2 not started"}), 503

    payload = request.get_json(silent=True) or {}

    try:
        lx = float(payload.get("linear_x", 0.0))
        ly = float(payload.get("linear_y", 0.0))
        az = float(payload.get("angular_z", 0.0))

        node.publish_cmd_vel(lx, ly, az)
        return jsonify({"success": True})
    except Exception as exc:
        return jsonify({"success": False, "error": str(exc)}), 400


@app.route("/api/camera/snapshot", methods=["GET"])
def api_camera_snapshot():
    if node is None or node.latest_camera is None:
        return jsonify({"success": False, "error": "No camera frame"}), 503

    with node.camera_lock:
        jpeg_bytes = bytes(node.latest_camera.data)

    return Response(jpeg_bytes, mimetype="image/jpeg")


@app.route("/api/camera/stream", methods=["GET"])
def api_camera_stream():
    if node is None:
        return jsonify({"success": False, "error": "ROS2 not started"}), 503

    boundary = "frame"

    def generator():
        while True:
            if node.latest_camera is None:
                time.sleep(0.1)
                continue
            with node.camera_lock:
                jpeg = bytes(node.latest_camera.data)

            yield (
                f"--{boundary}\r\n"
                "Content-Type: image/jpeg\r\n"
                f"Content-Length: {len(jpeg)}\r\n\r\n"
            ).encode() + jpeg + b"\r\n"

            time.sleep(0.03)  # ≈ 30 FPS max

    return Response(generator(), mimetype=f"multipart/x-mixed-replace; boundary={boundary}")


# ======================================================================
# ENTRY POINT
# ======================================================================

def main(args=None):
    start_ros(args=args)
    try:
        app.run(host=APP_HOST, port=APP_PORT, threaded=True, use_reloader=False)
    finally:
        shutdown_ros()


if __name__ == "__main__":
    main()
