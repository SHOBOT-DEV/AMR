# SHOBOT AMR API Endpoints Documentation

This document provides a comprehensive list of all API endpoints available in the SHOBOT AMR system.

---

## 📡 API Overview

The system has **4 main API interfaces**:

1. **Node.js Backend REST API** (Port 5000) - Main web API
2. **Python Analytics HTTP API** (Port 6060) - Robot analytics
3. **ROS Services** - Internal robot control APIs
4. **ROS Bridge WebSocket API** (Port 9090) - Real-time ROS communication

---

## 1. Node.js Backend REST API

**Base URL**: `http://<server-ip>:5000`  
**Authentication**: JWT Token (via `x-auth-token` header)

### Authentication Endpoints

#### POST `/api/v1/auth/`
Authenticate and get JWT token.

**Request Body**:
```json
{
  "email": "admin@shobot.com",
  "password": "shobot123"
}
```

**Response**:
```json
{
  "token": "<jwt-token>"
}
```

**Usage**: Used by `ui_interface` to authenticate and fetch data.

---

### Map Management Endpoints

#### GET `/api/v1/maps/data/?isActive=true`
Get active map data including map, waypoints, and zones.

**Headers**:
```
x-auth-token: <jwt-token>
```

**Response Structure**:
```json
{
  "map": {
    "name": "simulation_map",
    "properties": {
      "resolution": 0.05,
      "origin": {
        "position": {
          "x": -10.0,
          "y": -10.0
        }
      }
    }
  },
  "waypoints": [
    {
      "_id": "waypoint_id",
      "name": "Charging Station 1",
      "category": "CHARGING",
      "coordinate": {
        "position": {
          "x": 1.0,
          "y": 2.0
        },
        "orientation": {
          "z": 0.0,
          "w": 1.0
        }
      },
      "dockMarker": 0
    }
  ],
  "zones": [
    {
      "name": "Zone 1",
      "category": "RESTRICTED",
      "isActive": true,
      "geometry": {
        "type": "Polygon",
        "coordinates": [[[x1, y1], [x2, y2], ...]]
      }
    }
  ]
}
```

**Used by**: `shobot_interface/ui_interface` to load maps, waypoints, and zones.

---

### Additional Backend Endpoints (Inferred from Docker Config)

Based on the deployment configuration, the Node.js backend likely provides:

- **Mission Management**: `/api/v1/missions/*`
  - `GET /api/v1/missions/queued` - Get queued missions
  - `POST /api/v1/missions` - Create mission
  - `PUT /api/v1/missions/:id` - Update mission
  - `DELETE /api/v1/missions/:id` - Delete mission

- **Map Management**: `/api/v1/maps/*`
  - `GET /api/v1/maps` - List all maps
  - `POST /api/v1/maps` - Upload map
  - `DELETE /api/v1/maps/:id` - Delete map

- **Waypoint Management**: `/api/v1/waypoints/*`
  - CRUD operations for waypoints

- **Zone Management**: `/api/v1/zones/*`
  - CRUD operations for zones

- **Bag File Management**: `/api/v1/bags/*`
  - List, download, delete bag files

- **User Management**: `/api/v1/users/*`
  - User CRUD operations

**Note**: The actual Node.js backend code is in a Docker image (`guptaashwanee/anya-server`) and not in this repository.

---

## 2. C++ Analytics HTTP API

**Base URL**: `http://<robot-ip>:6060`  
**Server**: Boost.Beast HTTP server in `shobot_analytics`

### Analytics Endpoints

#### GET `/get_all_robot`
Get complete robot analytics data.

**Request**: `GET /get_all_robot`

**Response**: JSON with robot analytics data (implementation in progress)

**Configuration**: 
- Port: 6060 (hardcoded in `ApiInterface.cpp`)
- Parameter: `get_complete_robot_api` (default: `"get_all_robot"`)

**Location**: `catkin_ws/src/shobot_systems/shobot_analytics/src/ApiInterface.cpp`

---

## 3. ROS Services API

ROS Services are request-response APIs that can be called via:
- ROS command line: `rosservice call <service_name> <args>`
- ROS Bridge WebSocket (from web UI)
- Direct ROS service clients

### Navigation Services

#### `/navigation_server/go_to_pose`
Navigate robot to a specific pose.

**Service Type**: `shobot_msgs/GoToPose`

**Request**:
```
bool initiate
int16 level
```

**Response**:
```
bool status
```

---

#### `/navigation_server/update_pose`
Update robot's current pose.

**Service Type**: `shobot_msgs/updatePose`

---

### Map Management Services

#### `/reload`
Reload map from file.

**Service Type**: `shobot_msgs/MapReload`

**Request**:
```
string filename
```

**Response**:
```
bool is_active
string map_name
```

**Used by**: `ui_interface` to reload maps from backend API.

---

#### `/set_map`
Set active map.

**Service Type**: `shobot_msgs/SetMap`

---

#### `/get_map_name`
Get current map name.

**Service Type**: `shobot_msgs/GetMapName`

---

#### `/save_map`
Save current map.

**Service Type**: `shobot_msgs/SaveMap`

---

#### `/delete_map`
Delete a map.

**Service Type**: `shobot_msgs/DeleteMap`

---

#### `/map_crop`
Crop a map region.

**Service Type**: `shobot_msgs/MapCrop`

---

#### `/map_converter`
Convert map format.

**Service Type**: `shobot_msgs/MapConverter`

---

### Waypoint Services

#### `/waypoint_infos`
Get waypoint information by type.

**Service Type**: `shobot_msgs/WaypointInfos`

**Request**:
```
string type  # "charging", "home", or "general"
```

**Response**:
```
shobot_msgs/WaypointInfo[] poses
```

**WaypointInfo Structure**:
```
string id
string name
geometry_msgs/Pose pose
bool is_available
int32 type
```

**Used by**: Navigation system to get charging stations, home locations, etc.

---

#### `/ui/get_waypoint`
Get waypoint information (UI interface).

**Service Type**: `shobot_msgs/GetWayPoint`

---

### Zone Management Services

#### `/set_zone`
Set navigation zones.

**Service Type**: `shobot_msgs/SetZone`

**Request**:
```
shobot_msgs/ZonePoints[] zone_points
```

**Used by**: `ui_interface` to set zones from backend API.

---

#### `/clear_zones`
Clear all zones.

**Service Type**: `shobot_msgs/ClearZones`

---

#### `/get_transition_zones`
Get transition zones.

**Service Type**: `shobot_msgs/GetTransitionZones`

---

#### `/safety_zone`
Configure safety zones.

**Service Type**: `shobot_msgs/SafetyZone`

---

#### `/zone_speed`
Set speed limits for zones.

**Service Type**: `shobot_msgs/ZoneSpeed`

---

### Mission Control Services

#### `/mission_control/*`
Mission control services (specific service names depend on implementation).

---

### Analytics Services

#### `/robot_analytics`
Get robot analytics data.

**Service Type**: `shobot_msgs/RobotAnalytics`

---

#### `/mission_analytics`
Get mission analytics data.

**Service Type**: `shobot_msgs/MissionAnalytics`

---

#### `/battery_analytics`
Get battery analytics data.

**Service Type**: `shobot_msgs/BatteryAnalytics`

---

#### `/trip_analytics`
Get trip analytics data.

**Service Type**: `shobot_msgs/TripAnalytics`

---

### Safety Services

#### `/safety`
Safety control.

**Service Type**: `shobot_msgs/Safety`

---

#### `/safety_switch`
Control safety switch.

**Service Type**: `shobot_msgs/SafetySwitch`

---

### Charging Services

#### `/charger_location`
Get/set charger location.

**Service Type**: `shobot_msgs/ChargerLocation`

---

#### `/home_location`
Get/set home location.

**Service Type**: `shobot_msgs/HomeLocation`

---

#### `/charging_override`
Override charging behavior.

**Service Type**: `shobot_msgs/ChargingOverride`

---

#### `/set_charging_schedule`
Set charging schedule.

**Service Type**: `shobot_msgs/SetChargingSchedule`

---

### Hardware Control Services

#### `/lift_control`
Control lift mechanism.

**Service Type**: `shobot_msgs/LiftControl`

---

#### `/relay_state`
Control relay states.

**Service Type**: `shobot_msgs/RelayState`

---

#### `/dynamic_footprint`
Set dynamic robot footprint.

**Service Type**: `shobot_msgs/DynamicFootprint`

---

### Sensor Services

#### `/shelf_reader`
Read shelf/barcode data.

**Service Type**: `shobot_msgs/ShelfReader`

---

### Path Planning Services

#### `/path_updater`
Update navigation path.

**Service Type**: `shobot_msgs/PathUpdater`

---

#### `/nearest_node`
Find nearest graph node.

**Service Type**: `shobot_msgs/NearestNode`

---

### Database Services

#### `/database_management`
Database operations.

**Service Type**: `shobot_msgs/DataBaseManagement`

---

### Logging Services

#### `/log_cleaning`
Clean log files.

**Service Type**: `shobot_msgs/LogCleaning`

---

#### `/save_log_data`
Save log data.

**Service Type**: `log_recorder/SaveLogData`

---

### Bag File Services

#### `/delete_bags`
Delete ROS bag files.

**Service Type**: `shobot_msgs/DeleteBags`

---

### User Feedback Services

#### `/user_feedback`
Submit user feedback.

**Service Type**: `shobot_msgs/UserFeedback`

---

#### `/pausing_feedback`
Pausing feedback.

**Service Type**: `shobot_msgs/PausingFeedback`

---

### ROS Bridge Services (rosapi)

These services provide metadata about the ROS system and can be accessed via ROS Bridge:

#### `/rosapi/topics`
Get list of all ROS topics.

**Service Type**: `rosapi/Topics`

---

#### `/rosapi/services`
Get list of all ROS services.

**Service Type**: `rosapi/Services`

---

#### `/rosapi/nodes`
Get list of all ROS nodes.

**Service Type**: `rosapi/Nodes`

---

#### `/rosapi/topic_type`
Get type of a topic.

**Service Type**: `rosapi/TopicType`

---

#### `/rosapi/service_type`
Get type of a service.

**Service Type**: `rosapi/ServiceType`

---

#### `/rosapi/get_param`
Get ROS parameter.

**Service Type**: `rosapi/GetParam`

---

#### `/rosapi/set_param`
Set ROS parameter.

**Service Type**: `rosapi/SetParam`

---

#### `/rosapi/get_time`
Get ROS time.

**Service Type**: `rosapi/GetTime`

---

**Full list of rosapi services**:
- `Topics`, `TopicsForType`, `TopicsAndRawTypes`
- `Services`, `ServicesForType`
- `Nodes`, `NodeDetails`
- `Publishers`, `Subscribers`
- `ServiceProviders`, `ServiceHost`, `ServiceNode`
- `GetParam`, `SetParam`, `DeleteParam`, `GetParamNames`, `HasParam`, `SearchParam`
- `GetTime`
- `MessageDetails`, `ServiceRequestDetails`, `ServiceResponseDetails`
- `GetActionServers`

---

## 4. ROS Bridge WebSocket API

**URL**: `ws://<robot-ip>:9090` or `wss://<robot-ip>:9090`  
**Protocol**: rosbridge v2.0 JSON protocol

ROS Bridge allows web clients to interact with ROS via WebSocket using JSON messages.

### Publishing to ROS Topics

**Message Format**:
```json
{
  "op": "publish",
  "topic": "/cmd_vel",
  "msg": {
    "linear": {
      "x": 0.5,
      "y": 0.0,
      "z": 0.0
    },
    "angular": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0
    }
  }
}
```

---

### Subscribing to ROS Topics

**Subscribe**:
```json
{
  "op": "subscribe",
  "topic": "/scan",
  "type": "sensor_msgs/LaserScan"
}
```

**Unsubscribe**:
```json
{
  "op": "unsubscribe",
  "topic": "/scan"
}
```

**Incoming Messages**:
```json
{
  "op": "publish",
  "topic": "/scan",
  "msg": {
    "header": {...},
    "ranges": [...],
    ...
  }
}
```

---

### Calling ROS Services

**Call Service**:
```json
{
  "op": "call_service",
  "service": "/navigation_server/go_to_pose",
  "args": {
    "initiate": true,
    "level": 1
  }
}
```

**Service Response**:
```json
{
  "op": "service_response",
  "service": "/navigation_server/go_to_pose",
  "values": {
    "status": true
  },
  "result": true
}
```

---

### Common ROS Bridge Operations

- `publish` - Publish to a topic
- `subscribe` - Subscribe to a topic
- `unsubscribe` - Unsubscribe from a topic
- `call_service` - Call a ROS service
- `advertise` - Advertise a topic (for publishing)
- `unadvertise` - Stop advertising a topic
- `advertise_service` - Advertise a service
- `unadvertise_service` - Stop advertising a service

**Full Protocol Documentation**: See `catkin_ws/src/shobot_systems/rosbridge_suite/ROSBRIDGE_PROTOCOL.md`

---

## 4b. ROS2 Node Interfaces (this workspace)

ROS2 nodes in `SHOBOT_AMR_ws/src` expose these topics/services/actions by default (all parameters can be remapped):

- **shobot_navigation**: subscribes `goal_topic` (`/goal_pose`); publishes `status_topic` (`/navigation_status`) and `feedback_topic` (`/navigation_feedback`); service `cancel_navigation` (`std_srvs/Trigger`); forwards to Nav2 `navigate_to_pose` action.
- **shobot_navigation_server**: service `shobot_navigation_server/navigate` (`shobot_navigation_server/Navigate` with `request_id` + `PoseStamped`); publishes `/navigation_server/status`.
- **shobot_mission_handler**: subscribes `/mission_command` (JSON missions); publishes `/mission_queue`; service `clear_missions` (`std_srvs/Trigger`); status on `/mission_handler_status`.
- **shobot_mission_control**: subscribes `/mission_queue` (JSON); publishes `/mission_status`; service `cancel_mission` (`std_srvs/Trigger`); sends Nav2 NavigateToPose actions and optional `dock` action (`shobot_docking/Dock`).
- **shobot_docking**: action server `dock` (`shobot_docking/Dock` goal `start: bool`); publishes `/dock_status`; uses `/cmd_vel` + `/odom`.
- **shobot_twist_mux**: subscribes `/cmd_vel/safety`, `/cmd_vel/teleop`, `/cmd_vel/nav`; publishes muxed `output_topic` (`/cmd_vel`); listens to `/safety_stop` to zero output.
- **shobot_costmap_safety_layer**: subscribes `/scan`; publishes `/safety_stop` (`std_msgs/Bool`) when ranges under threshold.
- **shobot_trajectory_controller**: subscribes `/cmd_vel_raw`; publishes smoothed `/cmd_vel` with accel limits.
- **shobot_local_planner**: subscribes `/plan` (`nav_msgs/Path`) and `/odom`; publishes `/cmd_vel` toward next waypoint.
- **shobot_costmap_plugins (CostmapBuilder)**: subscribes `/obstacles` (`PointCloud2`); publishes occupancy grid `/shobot_costmap`.
- **shobot_pointcloud_assembler**: subscribes `input_topics` (default `/points_filtered` list); publishes combined `/points_assembled`.
- **shobot_yolo_detection (DepthDetector)**: publishes `/detection_info` (JSON detections with `depth_m`) and `detection_image` (annotated); reads RealSense directly or subscribes `color_topic`/`depth_topic`.
- **shobot_rosbridge_suite (RosbridgeAnnouncer)**: publishes `/rosbridge/info` with `{host, port, ws_url}` heartbeat for clients.
- **Motor control samples (`ROS package/`)**: `motor_controll.py`, `teleop_motor.py`, and `depth.py` subscribe `/cmd_vel`, `/scan`, `/camera/*` depth/rgb, publish `/detection_info`, encoder topics, and command serial motor drivers with stop/resume safety.

---

## 4c. Backend REST API (ANSER/ANYA-style)

Base prefix: `/api/v1` (set header `Authorization: Bearer <token>`; POST/PUT bodies as JSON). Total unique patterns: 56.

- **Auth**: `POST /auth`
- **Users**: `POST /users`, `GET /users`, `GET /users/{userId}`, `GET /users/me`, `PUT /users/{userId}`, `DELETE /users/{userId}`, `POST /users/reset-password/{userId}`
- **Maps**: `GET /maps`, `GET /maps/{mapId}`, `GET /maps/preview/{mapId}`, `GET /maps/data`, `POST /maps`, `PUT /maps/{mapId}`, `PUT /maps/edit/{mapId}?saveAsNew=true`, `DELETE /maps/{mapId}`, `POST /maps/load/{mapId}`
- **Waypoints**: `GET /waypoints`, `GET /waypoints/{id}`, `POST /waypoints`, `PUT /waypoints/{id}?updatePose=true`, `DELETE /waypoints/{id}`
- **Missions**: `GET /missions`, `GET /missions/{missionId}`, `POST /missions`, `POST /missions/initiate/{missionId}`, `PUT /missions/{missionId}`, `DELETE /missions/{missionId}`
- **Mission logs**: `GET /missions/logs`, `GET /missions/logs/{logId}/progress`
- **Robot bags**: `GET /robot/bags/files`, `GET /robot/bags/downloads/utils/point.js`, `GET /robot/bags/file`, `GET /robot/bags/list`
- **Robot goals**: `POST /robot/quickgoal`, `POST /robot/quickgoal/{waypointId}`
- **Zones**: `GET /zones`, `GET /zones/{zoneId}`, `POST /zones`, `PUT /zones/{zoneId}`, `DELETE /zones/{zoneId}`
- **Graphs**: `GET /graphs`, `GET /graphs/{graphId}`, `POST /graphs`, `PUT /graphs/{graphId}`, `DELETE /graphs/{graphId}`
- **Logs**: `GET /logs?from&to`, `GET /logs/export?from&to`, `POST /logs`
- **Analytics**: `GET /analytics/battery?from&to`
- **Registers**: `GET /registers`
- **Downloads**: `GET /downloads?from&to`, `GET /downloads/{filename}`
- **Root**: `GET /`, `GET /v1/abc`

---

## 5. ROS Actions API

ROS Actions are long-running tasks that can be monitored. Accessible via ROS Bridge or direct ROS clients.

### Navigation Actions

#### `/dock`
Docking action.

**Action Type**: `shobot_msgs/DockAction`

**Goal**:
```
geometry_msgs/Pose target_pose
string docking_type
```

**Feedback**:
```
string status
float32 progress
```

**Result**:
```
bool success
string message
```

---

### Other Actions

All action definitions are in `catkin_ws/src/shobot_systems/shobot_msgs/action/`:
- `DockAction`
- `GoToPoseAction`
- `MultiWaypointAction`
- And more...

---

## 📊 API Usage Examples

### Example 1: Authenticate and Get Map Data (from ROS Node)

```cpp
// From ui_interface/ApiInterface.cpp
// 1. Authenticate
POST http://localhost:5000/api/v1/auth/
Body: {"email": "admin@shobot.com", "password": "shobot123"}
Response: {"token": "..."}

// 2. Get map data
GET http://localhost:5000/api/v1/maps/data/?isActive=true
Header: x-auth-token: <token>
```

---

### Example 2: Control Robot via ROS Bridge (from Web UI)

```javascript
// Connect to ROS Bridge
var ros = new ROSLIB.Ros({
  url: 'ws://robot-ip:9090'
});

// Publish velocity command
var cmdVel = new ROSLIB.Topic({
  ros: ros,
  name: '/cmd_vel',
  messageType: 'geometry_msgs/Twist'
});

cmdVel.publish({
  linear: { x: 0.5, y: 0, z: 0 },
  angular: { x: 0, y: 0, z: 0 }
});

// Subscribe to laser scan
var scanListener = new ROSLIB.Topic({
  ros: ros,
  name: '/scan',
  messageType: 'sensor_msgs/LaserScan'
});

scanListener.subscribe(function(message) {
  console.log('Received scan:', message);
});

// Call service
var goToPose = new ROSLIB.Service({
  ros: ros,
  name: '/navigation_server/go_to_pose',
  serviceType: 'shobot_msgs/GoToPose'
});

var request = new ROSLIB.ServiceRequest({
  initiate: true,
  level: 1
});

goToPose.callService(request, function(result) {
  console.log('Result:', result);
});
```

---

### Example 3: Get Waypoints via ROS Service

```bash
# Get charging waypoints
rosservice call /waypoint_infos "type: 'charging'"

# Get home waypoints
rosservice call /waypoint_infos "type: 'home'"

# Get all waypoints
rosservice call /waypoint_infos "type: 'general'"
```

---

## 🔐 Authentication

### Node.js Backend API
- **Method**: JWT Token
- **Header**: `x-auth-token: <token>`
- **Get Token**: POST `/api/v1/auth/`

### ROS Services
- **Authentication**: None (internal ROS network)
- **Access Control**: Network-level (ROS nodes on same network)

### ROS Bridge
- **Authentication**: Optional (configured in rosbridge_server)
- **Default**: No authentication (for local network)

---

## 📝 Notes

1. **Node.js Backend**: The actual implementation is in Docker image `guptaashwanee/anya-server`. The endpoints listed are inferred from usage in the ROS codebase.

2. **Port Configuration**:
   - Node.js Backend: 5000 (configurable via `NODE_PORT` env var)
   - Analytics API: 6060 (hardcoded in C++ code)
   - ROS Bridge: 9090 (default, configurable)

3. **Service Discovery**: Use `rosservice list` to see all available ROS services at runtime.

4. **Topic Discovery**: Use `rostopic list` to see all available ROS topics at runtime.

5. **ROS Bridge Protocol**: Full protocol specification available in `rosbridge_suite/ROSBRIDGE_PROTOCOL.md`

---

## 🔗 Related Documentation

- **Codebase Structure**: See `CODEBASE_STRUCTURE.md`
- **ROS Bridge Protocol**: `catkin_ws/src/shobot_systems/rosbridge_suite/ROSBRIDGE_PROTOCOL.md`
- **Service Definitions**: `catkin_ws/src/shobot_systems/shobot_msgs/srv/`
- **Message Definitions**: `catkin_ws/src/shobot_systems/shobot_msgs/msg/`
- **Action Definitions**: `catkin_ws/src/shobot_systems/shobot_msgs/action/`
