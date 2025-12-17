#!/usr/bin/env python3
"""
Quick Start Example: FastAPI + ROS2 Integration
This demonstrates how to create a Python backend that bridges ROS2 and a React frontend.
"""

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
import json
import asyncio
from threading import Thread

# ============================================================================
# Pydantic Models for API
# ============================================================================


class PoseRequest(BaseModel):
    x: float
    y: float
    theta: float


class CmdVelRequest(BaseModel):
    linear_x: float
    angular_z: float


class RobotStatus(BaseModel):
    status: str
    battery: float
    position: dict
    velocity: dict


# ============================================================================
# ROS2 Node for Robot Communication
# ============================================================================


class RobotBridgeNode(Node):
    """ROS2 Node that handles robot communication"""

    def __init__(self):
        super().__init__("robot_bridge_node")

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, "/robot_pose", self.pose_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.battery_sub = self.create_subscription(
            Float32, "/battery_level", self.battery_callback, 10
        )

        # State
        self.current_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.current_scan = None
        self.battery_level = 100.0
        self.websocket_clients: List[WebSocket] = []

        self.get_logger().info("Robot Bridge Node initialized")

    def pose_callback(self, msg: PoseStamped):
        """Callback when robot pose is updated"""
        self.current_pose = {
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "theta": msg.pose.orientation.z,  # Simplified
        }
        self.broadcast_to_clients({"type": "pose", "data": self.current_pose})

    def scan_callback(self, msg: LaserScan):
        """Callback when laser scan is received"""
        # Downsample for web transmission
        self.current_scan = {
            "ranges": list(msg.ranges[::10]),  # Sample every 10th point
            "angle_min": msg.angle_min,
            "angle_max": msg.angle_max,
            "range_min": msg.range_min,
            "range_max": msg.range_max,
        }
        self.broadcast_to_clients({"type": "scan", "data": self.current_scan})

    def battery_callback(self, msg: Float32):
        """Callback when battery level is updated"""
        self.battery_level = msg.data
        self.broadcast_to_clients(
            {"type": "battery", "data": {"level": self.battery_level}}
        )

    def publish_cmd_vel(self, linear_x: float, angular_z: float):
        """Publish velocity command to robot"""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(
            f"Published cmd_vel: linear={linear_x}, angular={angular_z}"
        )

    def publish_goal(self, x: float, y: float, theta: float):
        """Publish navigation goal"""
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = float(theta)
        self.goal_pub.publish(msg)
        self.get_logger().info(f"Published goal: x={x}, y={y}, theta={theta}")

    def add_websocket_client(self, client: WebSocket):
        """Add WebSocket client for real-time updates"""
        self.websocket_clients.append(client)
        self.get_logger().info(
            f"WebSocket client connected. Total: {len(self.websocket_clients)}"
        )

    def remove_websocket_client(self, client: WebSocket):
        """Remove WebSocket client"""
        if client in self.websocket_clients:
            self.websocket_clients.remove(client)
            self.get_logger().info(
                f"WebSocket client disconnected. Total: {len(self.websocket_clients)}"
            )

    def broadcast_to_clients(self, message: dict):
        """Broadcast message to all WebSocket clients"""
        if not self.websocket_clients:
            return

        # Send to all clients (non-blocking)
        disconnected = []
        for client in self.websocket_clients:
            try:
                # Use asyncio to send (if in async context)
                asyncio.create_task(client.send_json(message))
            except Exception as e:
                self.get_logger().warn(f"Error sending to client: {e}")
                disconnected.append(client)

        # Remove disconnected clients
        for client in disconnected:
            self.remove_websocket_client(client)


# ============================================================================
# FastAPI Application
# ============================================================================

# Initialize ROS2
rclpy.init()
ros2_node = RobotBridgeNode()

# Create FastAPI app
app = FastAPI(
    title="SHOBOT AMR API", description="ROS2-based AMR Control API", version="2.0.0"
)

# CORS middleware for React frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # React dev server
        "http://localhost:5173",  # Vite dev server
        "http://localhost:8080",  # Alternative
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ============================================================================
# REST API Endpoints
# ============================================================================


@app.get("/")
async def root():
    """Root endpoint"""
    return {"message": "SHOBOT AMR ROS2 API", "version": "2.0.0", "status": "online"}


@app.get("/api/v1/status", response_model=RobotStatus)
async def get_status():
    """Get current robot status"""
    return RobotStatus(
        status="online",
        battery=ros2_node.battery_level,
        position=ros2_node.current_pose,
        velocity={"linear_x": 0.0, "angular_z": 0.0},  # Could track this
    )


@app.post("/api/v1/navigation/go_to_pose")
async def go_to_pose(pose: PoseRequest):
    """Navigate robot to a specific pose"""
    try:
        ros2_node.publish_goal(pose.x, pose.y, pose.theta)
        return {
            "success": True,
            "message": f"Navigation goal set: ({pose.x}, {pose.y}, {pose.theta})",
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/v1/control/cmd_vel")
async def send_cmd_vel(cmd: CmdVelRequest):
    """Send velocity command to robot"""
    try:
        ros2_node.publish_cmd_vel(cmd.linear_x, cmd.angular_z)
        return {
            "success": True,
            "message": f"Velocity command sent: linear={cmd.linear_x}, angular={cmd.angular_z}",
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/v1/sensors/scan")
async def get_latest_scan():
    """Get latest laser scan data"""
    if ros2_node.current_scan is None:
        raise HTTPException(status_code=404, detail="No scan data available")
    return ros2_node.current_scan


@app.get("/api/v1/robot/pose")
async def get_robot_pose():
    """Get current robot pose"""
    return ros2_node.current_pose


# ============================================================================
# WebSocket Endpoint for Real-time Communication
# ============================================================================


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket endpoint for real-time robot data and control"""
    await websocket.accept()
    ros2_node.add_websocket_client(websocket)

    try:
        # Send initial state
        await websocket.send_json(
            {
                "type": "connected",
                "data": {
                    "pose": ros2_node.current_pose,
                    "battery": ros2_node.battery_level,
                },
            }
        )

        while True:
            # Receive commands from React frontend
            data = await websocket.receive_json()

            if data.get("type") == "cmd_vel":
                # Handle velocity command
                ros2_node.publish_cmd_vel(
                    data.get("linear_x", 0.0), data.get("angular_z", 0.0)
                )
                await websocket.send_json(
                    {"type": "ack", "message": "cmd_vel received"}
                )

            elif data.get("type") == "goal":
                # Handle navigation goal
                ros2_node.publish_goal(
                    data.get("x", 0.0), data.get("y", 0.0), data.get("theta", 0.0)
                )
                await websocket.send_json({"type": "ack", "message": "goal received"})

            elif data.get("type") == "ping":
                # Heartbeat
                await websocket.send_json(
                    {"type": "pong", "timestamp": data.get("timestamp")}
                )

            # Process ROS2 callbacks (non-blocking)
            rclpy.spin_once(ros2_node, timeout_sec=0.01)

    except WebSocketDisconnect:
        ros2_node.remove_websocket_client(websocket)
        print(f"WebSocket client disconnected")


# ============================================================================
# Background Task for ROS2 Spinning
# ============================================================================


def ros2_spin_thread():
    """Background thread to spin ROS2 node"""
    while rclpy.ok():
        rclpy.spin_once(ros2_node, timeout_sec=0.1)


@app.on_event("startup")
async def startup_event():
    """Startup event - start ROS2 spinning thread"""
    print("Starting ROS2 spinning thread...")
    thread = Thread(target=ros2_spin_thread, daemon=True)
    thread.start()
    print("FastAPI server started with ROS2 integration")


@app.on_event("shutdown")
async def shutdown_event():
    """Shutdown event - cleanup ROS2"""
    print("Shutting down ROS2 node...")
    ros2_node.destroy_node()
    rclpy.shutdown()
    print("ROS2 shutdown complete")


# ============================================================================
# Main Entry Point
# ============================================================================

if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=5000, log_level="info")
