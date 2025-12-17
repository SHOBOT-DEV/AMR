# SHOBOT AMR Codebase Structure - Frontend & Backend Overview

## Executive Summary

This is a **ROS (Robot Operating System) based Autonomous Mobile Robot (AMR)** system with a **web-based frontend** and **distributed backend architecture**. The system uses Docker containers for deployment and communicates via ROS topics, services, and WebSockets.

---

## 🎨 FRONTEND ARCHITECTURE

### Frontend Deployment (Docker Container)
**Location**: `ShobotAnyaDeployment/docker-compose.yml`

The frontend is deployed as a **Docker container** (`guptaashwanee/anya-client`):
- **Ports**: 80 (HTTP), 443 (HTTPS)
- **Network**: `frontend` network
- **Technology**: Likely a React/Vue/Angular web application (not in this repo - deployed as Docker image)
- **Communication**: Connects to backend via REST API and WebSocket (rosbridge)

### Frontend-Backend Communication
1. **REST API**: Direct HTTP/HTTPS communication with the backend server
2. **WebSocket (rosbridge)**: Real-time ROS topic communication
   - Backend exposes `ROSBRIDGE_WEBSOCKET_URL` environment variable
   - Frontend uses `roslibjs` or similar library to connect to rosbridge

---

## 🔧 BACKEND ARCHITECTURE

### Backend Components

#### 1. **Backend Server (Node.js)**
**Location**: `ShobotAnyaDeployment/docker-compose.yml` (service: `server`)

- **Image**: `guptaashwanee/anya-server`
- **Port**: 5000
- **Technology**: Node.js (runs `node index.js`)
- **Database**: MongoDB (via `MONGO_URI`)
- **Features**:
  - REST API endpoints
  - JWT authentication (`JWT_PRIVATE_KEY`)
  - Map management (volume: `maps`)
  - Bag file management (volume: `bags`)
  - Downloads management (volume: `downloads`)
  - Connects to ROS via rosbridge WebSocket

#### 2. **ROS Backend (Robot Software Stack)**

The ROS backend is organized into **5 main packages** in `catkin_ws/src/`:

---

### 📦 ROS Package Structure

#### **A. shobot_interface/** - Communication & Protocol Layer
**Purpose**: Interfaces for external communication and protocols

**Key Components**:
- **`ui_interface/`**: ROS-Web interface bridge
  - `UiInterface.cpp`: Publishes robot data (scans, paths, markers) for web UI
  - `ApiInterface.cpp`: HTTP API server (port 6060) for robot data
  - `DiagnosticsTreeUpdater.cpp`: System diagnostics for UI

- **`fleet_interface/`**: Fleet management protocols
  - **VDA5050 Protocol**: Industry standard for AGV/AMR communication
    - `vda5050_proto_msgs/`: Protocol buffer definitions
    - `vda5050_ros_msgs/`: ROS message wrappers
  - **Task Manager**: Mission/order management
  - **Vehicle Interface**: Generic vehicle communication
  - **MQTT Connector**: MQTT protocol support

- **`third_party_integrations/`**: Integration with external systems
  - Navigation clients for various manufacturers (Hytech, Molex, Motherson, Samsung)

- **`shobot_teleop/`**: Teleoperation interface

#### **B. shobot_systems/** - Core System Services
**Purpose**: High-level system management and orchestration

**Key Components**:

1. **`rosbridge_suite/`**: **WebSocket bridge to ROS**
   - `rosbridge_server/`: WebSocket server for web clients
   - `rosbridge_library/`: Core JSON-to-ROS conversion
   - `rosapi/`: ROS metadata API (topics, services, parameters)
   - **Critical for frontend**: Enables web UI to publish/subscribe ROS topics

2. **`navigation_server/`**: Main navigation state machine
   - State machine: Idle, Move, Dock, Charge, Wait, Error, etc.
   - Mission execution
   - Database management (SQLite)

3. **`mission_control/`**: Mission planning and control
4. **`mission_handler/`**: Mission execution handler
5. **`waypoint_management/`**: Waypoint database and management
6. **`shobot_analytics/`**: Analytics and data collection
   - `ApiInterface.cpp`: HTTP server (port 6060) for robot analytics
7. **`status_aggregator/`**: Aggregates system status
8. **`diagnostics/`**: System health monitoring
   - Diagnostic aggregator, updater, analyzers
9. **`emergency_handler/`**: Emergency stop and safety
10. **`twist_mux/`**: Velocity command multiplexer
11. **`power_control/`**: Power management
12. **`log_recorder/`**: System logging
13. **`ros_bag_recorder/`**: ROS bag recording
14. **`system_config/`**: System configuration management
15. **`start_shobot/`**: System startup launcher

#### **C. shobot_navigation/** - Navigation & Motion Control
**Purpose**: Path planning, localization, and motion control

**Key Components**:

1. **`shobot_docking/`**: Docking controllers
   - Multiple docking methods: Fiducial, ICP, Pallet, Wall, Two-Point, etc.
   - CUDA-accelerated ICP matching

2. **`shobot_local_planner/`**: Local path planner
3. **`lower_level_controller/`**: Low-level motor control
4. **`navigation/`**: Navigation stack
   - `amcl-cuda/`: CUDA-accelerated AMCL (localization)
   - `map_server/`: 2D map server
   - `map_server_3d/`: 3D map server
5. **`mwp/`**: Multi-waypoint planner
   - Graph-based path planning
   - Hybrid graph planner
6. **`costmap_plugins/`**: Costmap plugins
7. **`costmap_safety_layer/`**: Safety layer for costmaps
8. **`dynamic_footprint/`**: Dynamic robot footprint
9. **`navigation_speed_control/`**: Speed control
10. **`trajectory_controller/`**: Trajectory following controller
11. **`roboteq_serial/`**: Roboteq motor controller interface

#### **D. shobot_perception-localization/** - Perception & SLAM
**Purpose**: Sensor processing, SLAM, and localization

**Key Components**:

1. **`shobot_slam/`**: SLAM implementation
   - Cartographer integration
   - GMapping support
   - RTAB-Map support

2. **`auto_localization/`**: Automatic localization
3. **`robot_localization/`**: Sensor fusion for localization
4. **`laser_filters/`**: LiDAR scan filtering
   - Box filter, intensity filter, sector filter, etc.
5. **`pointcloud_filter/`**: Point cloud filtering
6. **`pointcloud_assembler/`**: Multi-sensor point cloud assembly
7. **`pointcloud_to_laserscan/`**: Point cloud to laser scan conversion
8. **`scan_merger/`**: Multiple LiDAR scan merging
9. **`safety_layer/`**: Perception-based safety
10. **`zone_management/`**: Zone-based navigation
11. **`yolo_detection/`**: YOLO object detection
12. **`vision_opencv/`**: OpenCV integration
    - `cv_bridge/`: ROS-OpenCV bridge
    - `image_geometry/`: Image geometry utilities
13. **`localization_monitor/`**: Localization quality monitoring

#### **E. shobot_sensors/** - Sensor Drivers
**Purpose**: Hardware sensor interfaces

**Key Components**:

1. **`shobot_camera/`**: Camera interfaces
   - `realsense-ros/`: Intel RealSense camera
   - `camera_monitor/`: Camera health monitoring

2. **`zed_wrapper/`**: ZED stereo camera
3. **`zed_driver/`**: ZED camera driver
4. **`zed_nodelets/`**: ZED camera nodelets
5. **`ouster_ros/`**: Ouster LiDAR
   - Full Ouster SDK integration
6. **`pepperl_fuchs/`**: Pepperl+Fuchs LiDAR (R2000)
7. **`rplidar_ros/`**: RPLiDAR drivers
8. **`sick_safetyscanners/`**: SICK safety scanners
9. **`reader_modules/`**: Barcode/QR code readers
   - PGV reader
   - Shelf reader

---

## 🔄 DATA FLOW ARCHITECTURE

### Frontend → Backend Flow
```
Web Browser (Frontend)
    ↓ HTTP/HTTPS
Backend Server (Node.js) - Port 5000
    ↓ REST API / WebSocket
ROS Bridge Server (rosbridge_suite)
    ↓ ROS Topics/Services
ROS Nodes (shobot_* packages)
    ↓ Hardware Drivers
Physical Robot Hardware
```

### Backend → Frontend Flow
```
Physical Robot Hardware
    ↓ Sensor Data
ROS Sensor Drivers (shobot_sensors)
    ↓ ROS Topics
ROS Processing Nodes (shobot_perception-localization, shobot_navigation)
    ↓ ROS Topics
UI Interface Node (shobot_interface/ui_interface)
    ↓ ROS Topics / HTTP API
ROS Bridge Server (rosbridge_suite) / Analytics API
    ↓ WebSocket / HTTP
Backend Server (Node.js)
    ↓ HTTP/HTTPS / WebSocket
Web Browser (Frontend)
```

---

## 🗄️ DATABASE ARCHITECTURE

### MongoDB (Primary Database)
**Location**: `ShobotAnyaDeployment/docker-compose.yml` (service: `mongo0`)

- **Port**: 27017 (internal), 27018 (external)
- **Replica Set**: Configurable via environment
- **Data Volumes**:
  - `anyaDb0`: Main database
  - `maps`: Map storage
  - `downloads`: Download files
  - `bags`: ROS bag files

**Usage**:
- User management
- Mission/order storage
- Map metadata
- System configuration

### SQLite (Local Databases)
Used by various ROS nodes for local data:
- `navigation_server/`: Navigation state and missions
- `mission_control/`: Mission planning data
- `waypoint_management/`: Waypoint database
- `reader_modules/`: Barcode/QR code database

---

## 🌐 NETWORK ARCHITECTURE

### Docker Networks (from docker-compose.yml)

1. **`frontend`**: Frontend and backend server communication
2. **`backend`**: Backend server and ROS bridge communication
3. **`mongo`**: MongoDB cluster network

### Ports

- **80/443**: Frontend web server (HTTP/HTTPS)
- **5000**: Backend Node.js server
- **9090**: ROS Bridge WebSocket (typical default)
- **6060**: Analytics API (HTTP server in `shobot_analytics`)
- **27018**: MongoDB (external access)

---

## 🔌 COMMUNICATION PROTOCOLS

### 1. **ROS Topics** (Internal Robot Communication)
- Standard ROS 1 (Noetic) topics
- Custom message types in `shobot_msgs/`
- Topics for sensors, navigation, control, diagnostics

### 2. **ROS Services** (Request-Response)
- Service definitions in `shobot_msgs/srv/`
- Used for mission control, waypoint management, etc.

### 3. **ROS Actions** (Long-running Tasks)
- Action definitions in `shobot_msgs/action/`
- Used for docking, navigation, etc.

### 4. **WebSocket (rosbridge)**
- JSON-based protocol
- Allows web clients to interact with ROS
- Protocol: `ws://` or `wss://`

### 5. **REST API**
- Node.js backend: Port 5000
- Analytics API: Port 6060 (C++ HTTP server)
- JWT authentication

### 6. **MQTT** (Fleet Management)
- VDA5050 protocol over MQTT
- Vehicle-to-fleet communication

### 7. **gRPC** (Protocol Buffers)
- VDA5050 protocol definitions
- Vehicle data interface

---

## 🛠️ BUILD SYSTEM

### ROS Build System
- **Build Tool**: Catkin (ROS 1 Noetic)
- **Build Directory**: `catkin_ws/build/`
- **Devel Directory**: `catkin_ws/devel/`
- **Install Directory**: `catkin_ws/install/` (if used)

### CMake
- Each ROS package has `CMakeLists.txt`
- C++ packages compiled with CMake
- Python packages use `setup.py`

### Docker
- **Compose File**: `ShobotAnyaDeployment/docker-compose.yml`
- **Services**: client, server, mongo0, mongoinit

---

## 📁 KEY DIRECTORIES

```
AMR/
├── ShobotAnyaDeployment/     # Docker deployment configuration
│   ├── docker-compose.yml    # Main deployment file
│   └── mongo/                # MongoDB setup scripts
│
├── catkin_ws/                # ROS workspace
│   ├── src/                  # Source code
│   │   ├── shobot_interface/ # Communication layer
│   │   ├── shobot_systems/   # Core system services
│   │   ├── shobot_navigation/# Navigation & motion
│   │   ├── shobot_perception-localization/ # Perception & SLAM
│   │   └── shobot_sensors/   # Sensor drivers
│   ├── build/                # Build artifacts
│   └── devel/                # Development environment
│
├── carto_ws/                 # Cartographer SLAM workspace
│   └── src/                  # Cartographer source
│
├── cuPCL/                    # CUDA-accelerated PCL
│   ├── cuICP/               # CUDA ICP
│   ├── cuNDT/               # CUDA NDT
│   └── ...
│
├── grpc/                     # gRPC library (dependency)
├── opencv-4.5.0/            # OpenCV library
└── opencv_contrib-4.5.0/    # OpenCV contrib modules
```

---

## 🚀 DEPLOYMENT

### Starting the System

1. **ROS Nodes**: Started via launch files in `start_shobot/`
2. **Docker Services**: Started via `docker-compose up` in `ShobotAnyaDeployment/`
3. **Configuration**: Environment variables in `.env` file

### Typical Startup Sequence

1. MongoDB starts and initializes
2. Backend server (Node.js) starts and connects to MongoDB
3. Frontend container starts and connects to backend
4. ROS nodes start (via launch files)
5. ROS Bridge server starts
6. Frontend connects to ROS Bridge via WebSocket

---

## 🔐 SECURITY

- **JWT Authentication**: Backend uses JWT for API authentication
- **MongoDB Authentication**: Username/password protected
- **HTTPS**: Frontend supports HTTPS (port 443)
- **Network Isolation**: Docker networks separate frontend/backend/database

---

## 📊 MONITORING & DIAGNOSTICS

- **ROS Diagnostics**: `shobot_systems/diagnostics/`
- **System Statistics**: `robot_diagnostics/`
- **Logging**: `log_recorder/`
- **Analytics**: `shobot_analytics/`
- **Status Aggregation**: `status_aggregator/`

---

## 🎯 SUMMARY

**Frontend**: Web application (Docker container) - communicates via HTTP/HTTPS and WebSocket

**Backend**: 
- **Node.js Server**: REST API, authentication, database management
- **ROS Stack**: Robot control, navigation, perception, sensor drivers
- **ROS Bridge**: WebSocket bridge connecting web to ROS
- **MongoDB**: Primary database for missions, maps, users

**Communication Flow**: 
Web UI ↔ Node.js Backend ↔ ROS Bridge ↔ ROS Nodes ↔ Robot Hardware

This is a **production-grade AMR system** with comprehensive navigation, perception, fleet management, and web-based control capabilities.

---

## 📚 Additional Documentation

- **API Endpoints**: See `API_ENDPOINTS.md` for complete API documentation including:
  - Node.js Backend REST API endpoints
  - C++ Analytics HTTP API
  - ROS Services API
  - ROS Bridge WebSocket API
  - Usage examples and authentication details

