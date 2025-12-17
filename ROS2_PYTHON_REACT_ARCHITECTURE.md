# ROS2 + Python + React Architecture Guide

## ✅ Yes, You Can Use ROS2 with Python Endpoints and React UI!

This guide explains how to architect an AMR system using **ROS2**, **Python** for backend endpoints, and **React** for the frontend UI.

---

## 🏗️ Architecture Overview

### Recommended Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    React Frontend (Web UI)                   │
│  - React Components                                          │
│  - State Management (Redux/Zustand)                         │
│  - Real-time Updates via WebSocket                          │
└──────────────────────┬──────────────────────────────────────┘
                       │ HTTP/HTTPS + WebSocket
                       │
┌──────────────────────▼──────────────────────────────────────┐
│          FastAPI/Flask Backend (Python)                      │
│  - REST API Endpoints                                        │
│  - WebSocket Server (for real-time ROS data)                │
│  - Authentication (JWT)                                      │
│  - Business Logic                                            │
└──────────────────────┬──────────────────────────────────────┘
                       │ ROS2 Python API (rclpy)
                       │
┌──────────────────────▼──────────────────────────────────────┐
│              ROS2 Middleware (DDS)                           │
│  - Topics, Services, Actions                                 │
│  - Discovery & Communication                                 │
└──────────────────────┬──────────────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────────────┐
│          ROS2 Nodes (Python/C++)                             │
│  - Navigation Nodes                                          │
│  - Sensor Drivers                                            │
│  - Control Nodes                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 🔄 Comparison: ROS1 vs ROS2

### Current System (ROS1)
- **Build System**: Catkin
- **Master Node**: Required (single point of failure)
- **Python API**: `rospy`
- **Web Bridge**: rosbridge_suite (ROS1)
- **Language**: Primarily C++ with some Python

### Proposed System (ROS2)
- **Build System**: Colcon (or ament)
- **Master Node**: None (decentralized DDS)
- **Python API**: `rclpy` (more Pythonic, async support)
- **Web Bridge**: ros2-web-bridge or custom FastAPI bridge
- **Language**: Python-first approach

---

## 📦 Key Components

### 1. ROS2 Python Backend API

#### Option A: FastAPI with ROS2 Integration (Recommended)

**Why FastAPI?**
- Modern, fast Python web framework
- Built-in WebSocket support
- Automatic API documentation (OpenAPI/Swagger)
- Async/await support (works well with ROS2 async)
- Type hints and validation

**Example Structure**:

```python
# backend/main.py
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Twist
from geometry_msgs.msg import PoseStamped
import json

app = FastAPI(title="SHOBOT AMR API")

# CORS for React frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # React dev server
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ROS2 Node Manager
class ROS2Manager(Node):
    def __init__(self):
        super().__init__('api_bridge_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/robot_pose', self.pose_callback, 10)
        self.connected_clients = []
    
    def pose_callback(self, msg):
        # Broadcast to all WebSocket clients
        pose_data = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.orientation.z
        }
        for client in self.connected_clients:
            client.send_json({'type': 'pose', 'data': pose_data})
    
    def publish_cmd_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

# Initialize ROS2
rclpy.init()
ros2_manager = ROS2Manager()

# REST API Endpoints
@app.get("/api/v1/status")
async def get_status():
    """Get robot status"""
    # Query ROS2 topics/services
    return {
        "status": "online",
        "battery": 85.0,
        "position": {"x": 1.0, "y": 2.0}
    }

@app.post("/api/v1/navigation/go_to_pose")
async def go_to_pose(x: float, y: float, theta: float):
    """Navigate to pose"""
    # Call ROS2 service or publish goal
    # Implementation here
    return {"success": True, "message": "Navigation started"}

@app.get("/api/v1/maps")
async def get_maps():
    """Get available maps"""
    # Query from database or ROS2 parameter server
    return {"maps": ["map1", "map2"]}

# WebSocket for real-time data
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    ros2_manager.connected_clients.append(websocket)
    
    try:
        while True:
            # Receive commands from React
            data = await websocket.receive_json()
            
            if data['type'] == 'cmd_vel':
                ros2_manager.publish_cmd_vel(
                    data['linear_x'], 
                    data['angular_z']
                )
            elif data['type'] == 'subscribe':
                # Subscribe to ROS2 topic
                pass
            
            # Spin ROS2 once to process callbacks
            rclpy.spin_once(ros2_manager, timeout_sec=0.01)
            
    except WebSocketDisconnect:
        ros2_manager.connected_clients.remove(websocket)

# Background task for ROS2 spinning
import asyncio
async def ros2_spin_task():
    while True:
        rclpy.spin_once(ros2_manager, timeout_sec=0.1)
        await asyncio.sleep(0.01)

@app.on_event("startup")
async def startup_event():
    asyncio.create_task(ros2_spin_task())

@app.on_event("shutdown")
async def shutdown_event():
    ros2_manager.destroy_node()
    rclpy.shutdown()
```

---

#### Option B: Flask with ROS2 Integration

```python
# backend/app.py
from flask import Flask, jsonify, request
from flask_socketio import SocketIO, emit
import rclpy
from rclpy.node import Node

app = Flask(__name__)
app.config['SECRET_KEY'] = 'your-secret-key'
socketio = SocketIO(app, cors_allowed_origins="*")

class ROS2Bridge(Node):
    def __init__(self):
        super().__init__('flask_ros2_bridge')
        # ROS2 publishers/subscribers here

rclpy.init()
ros2_bridge = ROS2Bridge()

@app.route('/api/v1/status', methods=['GET'])
def get_status():
    return jsonify({"status": "online"})

@socketio.on('connect')
def handle_connect():
    emit('connected', {'data': 'Connected to ROS2 bridge'})

@socketio.on('cmd_vel')
def handle_cmd_vel(data):
    # Publish to ROS2
    pass

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000)
```

---

### 2. React Frontend

#### React + ROS2 WebSocket Client

```typescript
// frontend/src/services/ros2Service.ts
import { io, Socket } from 'socket.io-client';

class ROS2Service {
  private socket: Socket | null = null;
  private subscribers: Map<string, (data: any) => void> = new Map();

  connect(url: string = 'ws://localhost:5000/ws') {
    this.socket = io(url, {
      transports: ['websocket'],
    });

    this.socket.on('connect', () => {
      console.log('Connected to ROS2 backend');
    });

    this.socket.on('message', (data: any) => {
      if (data.type && this.subscribers.has(data.type)) {
        this.subscribers.get(data.type)!(data.data);
      }
    });
  }

  subscribe(topic: string, callback: (data: any) => void) {
    this.subscribers.set(topic, callback);
    this.socket?.emit('subscribe', { topic });
  }

  publishCmdVel(linearX: number, angularZ: number) {
    this.socket?.emit('cmd_vel', {
      type: 'cmd_vel',
      linear_x: linearX,
      angular_z: angularZ,
    });
  }

  async goToPose(x: number, y: number, theta: number) {
    const response = await fetch('http://localhost:5000/api/v1/navigation/go_to_pose', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ x, y, theta }),
    });
    return response.json();
  }
}

export const ros2Service = new ROS2Service();
```

#### React Component Example

```tsx
// frontend/src/components/RobotControl.tsx
import React, { useEffect, useState } from 'react';
import { ros2Service } from '../services/ros2Service';

const RobotControl: React.FC = () => {
  const [pose, setPose] = useState({ x: 0, y: 0, theta: 0 });
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    ros2Service.connect();
    setConnected(true);

    // Subscribe to robot pose
    ros2Service.subscribe('pose', (data) => {
      setPose(data);
    });

    return () => {
      // Cleanup
    };
  }, []);

  const handleMove = (linearX: number, angularZ: number) => {
    ros2Service.publishCmdVel(linearX, angularZ);
  };

  return (
    <div>
      <h2>Robot Control</h2>
      <p>Status: {connected ? 'Connected' : 'Disconnected'}</p>
      <p>Position: X={pose.x}, Y={pose.y}, θ={pose.theta}</p>
      
      <button onClick={() => handleMove(0.5, 0)}>Move Forward</button>
      <button onClick={() => handleMove(0, 0.5)}>Turn Left</button>
      <button onClick={() => handleMove(0, -0.5)}>Turn Right</button>
      <button onClick={() => handleMove(0, 0)}>Stop</button>
    </div>
  );
};

export default RobotControl;
```

---

## 🔌 ROS2 Web Bridge Options

### Option 1: Custom FastAPI Bridge (Recommended)
- Full control over API design
- Direct ROS2 integration
- Best performance
- **Implementation**: See FastAPI example above

### Option 2: ros2-web-bridge
- Official ROS2 web bridge
- Similar to rosbridge_suite for ROS1
- Uses WebSocket JSON protocol

**Installation**:
```bash
npm install ros2-web-bridge
```

**Usage**:
```javascript
// In your Node.js backend or React
const ros2bridge = require('ros2-web-bridge');
const bridge = ros2bridge.createBridge({
  ros2WebSocketServer: {
    port: 9090
  }
});
```

### Option 3: rosbridge_suite (ROS1) → Migrate to ROS2
- If you need compatibility with existing rosbridge clients
- Requires ROS1-ROS2 bridge (ros1_bridge package)

---

## 📁 Project Structure

```
shobot-ros2/
├── backend/                    # Python FastAPI backend
│   ├── app/
│   │   ├── __init__.py
│   │   ├── main.py            # FastAPI app
│   │   ├── ros2_bridge.py     # ROS2 integration
│   │   ├── api/
│   │   │   ├── __init__.py
│   │   │   ├── navigation.py  # Navigation endpoints
│   │   │   ├── maps.py        # Map endpoints
│   │   │   └── status.py      # Status endpoints
│   │   ├── models/            # Pydantic models
│   │   └── services/          # Business logic
│   ├── requirements.txt
│   └── Dockerfile
│
├── frontend/                   # React frontend
│   ├── src/
│   │   ├── components/
│   │   ├── services/
│   │   │   └── ros2Service.ts
│   │   ├── App.tsx
│   │   └── main.tsx
│   ├── package.json
│   └── Dockerfile
│
├── ros2_ws/                    # ROS2 workspace
│   └── src/
│       ├── shobot_navigation/  # ROS2 navigation nodes
│       ├── shobot_sensors/     # ROS2 sensor drivers
│       └── shobot_msgs/        # ROS2 message definitions
│
├── docker-compose.yml
└── README.md
```

---

## 🚀 Migration Path from ROS1 to ROS2

### Step 1: Set Up ROS2 Environment

```bash
# Install ROS2 (Humble or Iron)
sudo apt install ros-humble-desktop

# Create ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Step 2: Convert Messages

**ROS1** (`shobot_msgs/GoToPose.srv`):
```
bool initiate
int16 level
---
bool status
```

**ROS2** (`shobot_msgs/srv/GoToPose.srv`):
```
bool initiate
int16 level
---
bool status
```

*Note: Service format is the same, but package structure differs*

### Step 3: Convert Python Nodes

**ROS1**:
```python
import rospy
from shobot_msgs.srv import GoToPose

def go_to_pose_client():
    rospy.wait_for_service('go_to_pose')
    try:
        go_to_pose = rospy.ServiceProxy('go_to_pose', GoToPose)
        resp = go_to_pose(True, 1)
        return resp.status
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
```

**ROS2**:
```python
import rclpy
from rclpy.node import Node
from shobot_msgs.srv import GoToPose

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self.client = self.create_client(GoToPose, 'go_to_pose')
    
    def go_to_pose(self, initiate: bool, level: int):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        request = GoToPose.Request()
        request.initiate = initiate
        request.level = level
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().status
```

### Step 4: Update Build System

**ROS1** (`CMakeLists.txt`):
```cmake
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  shobot_msgs
)
```

**ROS2** (`setup.py` or `CMakeLists.txt`):
```python
# setup.py
from setuptools import setup
from glob import glob

setup(
    name='shobot_navigation',
    version='0.0.0',
    packages=['shobot_navigation'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/shobot_navigation']),
        ('share/shobot_navigation', ['package.xml']),
        ('share/shobot_navigation/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='SHOBOT Navigation Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node = shobot_navigation.navigation_node:main',
        ],
    },
)
```

---

## 🔧 Key ROS2 Python Packages

```python
# requirements.txt for backend
fastapi==0.104.1
uvicorn[standard]==0.24.0
websockets==12.0
rclpy==3.3.0  # ROS2 Python client library
rclpy-message-filters  # For message synchronization
pydantic==2.5.0  # Data validation
python-jose[cryptography]  # JWT authentication
passlib[bcrypt]  # Password hashing
python-multipart  # File uploads
```

---

## 📊 Advantages of ROS2 + Python + React

### ✅ Benefits

1. **Modern Stack**
   - ROS2: Better performance, no master node, DDS-based
   - Python: Easier development, rich ecosystem
   - React: Modern UI framework, great developer experience

2. **Better Python Support**
   - ROS2 Python API is more Pythonic
   - Async/await support
   - Better type hints

3. **No Master Node**
   - Decentralized architecture
   - No single point of failure
   - Better for distributed systems

4. **Type Safety**
   - FastAPI with Pydantic for backend
   - TypeScript for frontend
   - Better IDE support

5. **Performance**
   - DDS middleware is faster than ROS1
   - Better real-time capabilities
   - Lower latency

### ⚠️ Considerations

1. **Migration Effort**
   - Need to rewrite ROS1 nodes to ROS2
   - Message/service definitions need conversion
   - Testing required

2. **Ecosystem Maturity**
   - Some ROS1 packages may not have ROS2 equivalents
   - May need to port dependencies

3. **Learning Curve**
   - Team needs to learn ROS2 concepts
   - Different API patterns

---

## 🎯 Recommended Implementation Strategy

### Phase 1: Proof of Concept
1. Set up ROS2 workspace
2. Create simple FastAPI backend with ROS2 integration
3. Build basic React UI
4. Test with one ROS2 node (e.g., cmd_vel publisher)

### Phase 2: Core Features
1. Implement navigation API endpoints
2. Add WebSocket for real-time data
3. Build React components for robot control
4. Add authentication

### Phase 3: Full Migration
1. Port all ROS1 nodes to ROS2
2. Migrate all services
3. Update message definitions
4. Comprehensive testing

### Phase 4: Production
1. Docker containerization
2. CI/CD pipeline
3. Monitoring and logging
4. Documentation

---

## 📚 Resources

- **ROS2 Documentation**: https://docs.ros.org/
- **FastAPI Documentation**: https://fastapi.tiangolo.com/
- **React Documentation**: https://react.dev/
- **rclpy API**: https://docs.ros2.org/latest/api/rclpy/
- **ros2-web-bridge**: https://github.com/RobotWebTools/ros2-web-bridge

---

## 🔗 Integration with Existing System

You can run ROS1 and ROS2 side-by-side using `ros1_bridge`:

```bash
# Install ros1_bridge
sudo apt install ros-humble-ros1-bridge

# Run bridge
ros2 run ros1_bridge dynamic_bridge
```

This allows gradual migration while maintaining compatibility.

---

## 💡 Example: Complete FastAPI + ROS2 + React Setup

See the code examples above for:
- FastAPI backend with ROS2 integration
- React frontend with WebSocket client
- Real-time robot control
- REST API endpoints

This architecture provides a modern, scalable, and maintainable solution for your AMR system!

