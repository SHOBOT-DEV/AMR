shobot_navigation_server

A lightweight wrapper around Nav2 that exposes a simple ROS 2 service interface for sending navigation goals, hiding the complexity of Nav2 action handling from external callers (FSMs, mission planners, web UIs, or fleet managers).

This node acts as a bridge between high-level mission logic and Nav2’s NavigateToPose action.

Launch
ros2 launch shobot_navigation_server shobot_navigation_server_launch.py


Make sure Nav2 is already running (bringup, map server, localization) before launching this node.

Features

Exposes a simple ROS 2 service to trigger navigation

Internally forwards requests to Nav2 NavigateToPose action

Publishes continuous navigation status updates

Supports:

Goal cancellation

Optional goal timeout

Feedback throttling

Designed for FSM / mission-handler friendly integration

Service Interface
Navigate Service

Service name

shobot_navigation_server/navigate


Service type

shobot_navigation_server/srv/Navigate.srv

Expected Navigate.srv definition
string request_id
geometry_msgs/PoseStamped pose
---
bool success
string message


request_id
User-defined identifier for tracking navigation requests

pose
Target goal pose (map frame recommended)

success
Indicates whether the request was accepted

message
Human-readable status or error message

Published Topics
Navigation Status

Topic

/navigation_server/status


Type

std_msgs/String


Description

Publishes high-level navigation status messages such as:

Goal accepted / rejected

Feedback (distance remaining, speed)

Navigation succeeded / aborted / canceled

Timeout or cancellation events

This topic is ideal for:

Mission handlers

UI dashboards

Logging and debugging

Fleet coordination logic

Parameters
Parameter	Type	Default	Description
nav2_action_name	string	/navigate_to_pose	Nav2 action name
status_topic	string	/navigation_server/status	Status output topic
feedback_throttle_sec	float	0.5	Minimum interval between feedback updates
goal_timeout_sec	float	0.0	Max navigation duration (0 = disabled)
Usage Example
Call the Navigation Service
ros2 service call /shobot_navigation_server/navigate shobot_navigation_server/srv/Navigate \
"{ request_id: 'goal_001',
   pose:
     { header: { frame_id: 'map' },
       pose:
         { position: { x: 2.0, y: 1.0, z: 0.0 },
           orientation: { w: 1.0 } } } }"

Typical Architecture
Mission Handler / FSM
        |
        |  Navigate.srv
        v
shobot_navigation_server
        |
        |  NavigateToPose action
        v
        Nav2 Stack

Design Notes

This node does not replace Nav2

It simplifies external control and orchestration

Only one active goal is supported at a time

New requests automatically cancel any active goal

Status messages are human-readable by design

Recommended Use Cases

Autonomous mission execution

Task sequencing (pick → navigate → dock)

Web-based robot control panels

Fleet management systems

Research platforms needing simplified navigation APIs

Related Packages

nav2_bringup – Core Nav2 navigation stack

shobot_mission_handler – High-level task orchestration

shobot_fsm – State machine control

shobot_status_aggregator – System-wide status reporting