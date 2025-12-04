#!/usr/bin/env python3
import asyncio
from threading import Thread
from typing import List

import rclpy
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

app = FastAPI(title="SHOBOT API Bridge", version="0.1.0")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


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
        self.connected_ws: List[WebSocket] = []
        self.latest_status = {}
        self.get_logger().info(
            f"Bridge running (cmd_vel: {cmd_vel_topic}, status: {status_topic})"
        )

    def status_callback(self, msg):
        # Store latest status; extend with real fields when available.
        self.latest_status = {"vx": msg.linear.x, "vy": msg.linear.y, "wz": msg.angular.z}

    def publish_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.linear.y = float(linear_y)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)


node: ApiBridge | None = None
executor: MultiThreadedExecutor | None = None
spin_thread: Thread | None = None


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


@app.get("/api/status")
async def get_status():
    if node is None:
        return {"status": "offline"}
    return {"status": "online", "latest_status": node.latest_status}


@app.post("/api/cmd_vel")
async def set_cmd_vel(linear_x: float = 0.0, linear_y: float = 0.0, angular_z: float = 0.0):
    if node is None:
        return {"success": False, "error": "ROS2 not started"}
    node.publish_cmd_vel(linear_x, linear_y, angular_z)
    return {"success": True}


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    if node is None:
        await websocket.close(code=1011)
        return

    await websocket.accept()
    node.connected_ws.append(websocket)
    try:
        while True:
            try:
                data = await asyncio.wait_for(websocket.receive_json(), timeout=0.05)
            except asyncio.TimeoutError:
                # Periodically send status
                await websocket.send_json({"type": "status", "data": node.latest_status})
                continue

            if data.get("type") == "cmd_vel":
                payload = data.get("data", {})
                node.publish_cmd_vel(
                    payload.get("linear_x", 0.0),
                    payload.get("linear_y", 0.0),
                    payload.get("angular_z", 0.0),
                )
                await websocket.send_json({"type": "ack", "cmd": "cmd_vel"})
    except WebSocketDisconnect:
        pass
    finally:
        if websocket in node.connected_ws:
            node.connected_ws.remove(websocket)


def main():
    import uvicorn

    start_ros()
    try:
        uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")
    finally:
        shutdown_ros()


if __name__ == "__main__":
    main()
