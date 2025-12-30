# shobot_rosbridge_suite

WebSocket-based ROS 2 bridge for **SHOBOT Autonomous Mobile Robots (AMR)**.

This package enables **JSON/WebSocket access** to ROS 2 topics, services, and parameters for:
- Web dashboards
- Fleet management systems
- Remote monitoring and control
- External UI applications (React, Vue, Flutter, etc.)

It is built on top of **rosbridge_server** and includes a custom **endpoint announcer** for discovery.

---

## Overview

`shobot_rosbridge_suite` provides:

1. **Rosbridge WebSocket Server**
   - Exposes ROS topics and services over WebSocket
   - Enables JSON-based publish/subscribe

2. **Rosbridge Announcer**
   - Publishes connection metadata (`ws://host:port`)
   - Useful for dashboards and auto-discovery
   - Optional heartbeat support

---

## Components

### 1. Rosbridge Server

The standard rosbridge WebSocket server allows external clients to:

- Subscribe to ROS topics
- Publish ROS messages
- Call ROS services
- Interact using JSON over WebSocket

This is typically used by:
- Web UIs
- Cloud backends
- Fleet management tools

---

### 2. Rosbridge Announcer (`shobot_rosbridge_announcer`)

Publishes rosbridge connection information on a ROS topic.

**Published Information**
- Host
- Port
- WebSocket URL
- Node name
- Timestamp
- Connection status

This allows UIs to **auto-discover** the correct WebSocket endpoint without hardcoding addresses.

---

## Topics

### Published Topics

| Topic | Type | Description |
|------|------|------------|
| `/rosbridge/info` | `std_msgs/String` (JSON) | WebSocket endpoint metadata |

Example payload:
```json
{
  "host": "192.168.1.20",
  "port": 9090,
  "ws_url": "ws://192.168.1.20:9090",
  "node": "shobot_rosbridge_announcer",
  "status": "online",
  "timestamp_ns": 1700000000000000000
}

```
```bash
  ros2 launch shobot_rosbridge_suite shobot_rosbridge_suite_launch.py
```

```bash
  ros2 run shobot_rosbridge_suite shobot_rosbridge_announcer_node \
  --ros-args -p host:=192.168.1.20 -p port:=9090

```

```text
ROS 2 System
     ↓
rosbridge_server (WebSocket)
     ↓
shobot_rosbridge_announcer
     ↓
/rosbridge/info
     ↓
Web Dashboard / Fleet UI

```
## JavaScript Client
```javascript
const ws = new WebSocket("ws://192.168.1.20:9090");

ws.onopen = () => {
  ws.send(JSON.stringify({
    op: "subscribe",
    topic: "/system/heartbeat"
  }));
};

```