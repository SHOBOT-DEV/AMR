#!/usr/bin/env python3
"""
Motor control node that fuses LiDAR and camera detections to stop or resume
motors. Subscribes to `/cmd_vel`, `/scan`, YOLO detections on `/camera/*`, and
publishes detection info on `/detection_info`. Commands left/right motors over
serial after converting linear/angular velocity to RPM. Ported to ROS2.
"""
import rclpy
from rclpy.node import Node
import serial
import struct
import time
import json
from math import isnan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from sensor_msgs.msg import PointCloud2
import numpy as np
import torch
from ultralytics import YOLO  # Make sure to install ultralytics
import cv2
from cv_bridge import CvBridge

# Serial communication settings
ser = serial.Serial(
    port='/dev/ttyUSB0',  # Replace with your actual COM port
    baudrate=19200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

# Constants
WHEEL_RADIUS = 0.08  # meters
WHEELBASE = 0.447967  # meters
MAX_RPM = 3000  # Motor's max speed
LIDAR_THRESHOLD = 0.1  # LiDAR detection threshold in meters
CAMERA_THRESHOLD = 0.5  # Camera detection threshold in meters

# Utility Functions
def crc16(data: bytes):
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if crc & 1:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc

def send_command(command: bytes):
    crc_value = crc16(command)
    crc_bytes = struct.pack('<H', crc_value)
    frame = command + crc_bytes
    ser.write(frame)
    time.sleep(0.1)
    response = ser.read(ser.in_waiting)

def enable_motor():
    enable_command = b'\x01\x06\x10\x00\x00\x0A'
    send_command(enable_command)

def set_motor_speed(motor: str, speed: int):
    speed_bytes = struct.pack('>i', speed)
    high_word = speed_bytes[:2]
    low_word = speed_bytes[2:]
    corrected_speed_bytes = low_word + high_word

    if motor == 'left':
        command = b'\x01\x10\x10\x04\x00\x02\x04' + corrected_speed_bytes
    elif motor == 'right':
        command = b'\x01\x10\x10\x06\x00\x02\x04' + corrected_speed_bytes
    else:
        # Logging happens inside node methods; keep a silent guard here.
        return

    send_command(command)

def mps_to_rpm(linear_velocity):
    return (linear_velocity * 60) / (2 * 3.14159 * WHEEL_RADIUS)

def radps_to_rpm(angular_velocity):
    return (angular_velocity * WHEELBASE * 60) / (2 * 3.14159 * WHEEL_RADIUS)

def limit_speed(speed):
    return max(min(speed, MAX_RPM), -MAX_RPM)

class MotorControlNode(Node):
    """Handles motor commands plus obstacle checks from LiDAR and YOLO camera."""
    def __init__(self):
        super().__init__('motor_control_node')

        self.previous_left_speed = 0
        self.previous_right_speed = 0
        self.object_detected = False
        self.lidar_closest_distance = float('inf')
        self.camera_closest_distance = float('inf')

        self.frame_counter = 0

        # Subscribers
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)
        self.create_subscription(String, "/detection_info", self.camera_callback, 10)
        self.create_subscription(Image, "/camera/rgb/image_raw", self.image_callback, 10)
        self.create_subscription(Image, "/camera/depth/image_raw", self.depth_callback, 10)

        # Publisher for detection info
        self.detection_pub = self.create_publisher(String, '/detection_info', 10)

        enable_motor()

        # Load the YOLOv8 model and set it to use CUDA for inference
        self.model = YOLO("yolov8n.pt")  # Load the YOLO model (make sure it's in the right path)

        # Initialize CvBridge for converting ROS Image messages to OpenCV format
        self.bridge = CvBridge()

    def lidar_callback(self, msg):
        """Sample LiDAR data and record the closest obstacle in the front arc."""
        self.frame_counter += 1

        if self.frame_counter % 5 != 0:
            return

        self.frame_counter = 0

        num_readings = len(msg.ranges)
        half_range = num_readings // 2
        front_180_deg_data = msg.ranges[:half_range]
        self.lidar_closest_distance = float('inf')

        for distance in front_180_deg_data:
            if isnan(distance):
                self.get_logger().warn("NaN value detected in LiDAR data!")
                continue
            if distance < self.lidar_closest_distance:
                self.lidar_closest_distance = distance

        self.get_logger().info(f"Lidar Front 180° Distance: {self.lidar_closest_distance}")
        self.check_obstacle()

    def camera_callback(self, msg):
        """Update camera-derived closest distance from YOLO depth metadata."""
        try:
            detection_data = json.loads(msg.data)
            closest = float('inf')

            # Preferred format: list of detections with depth_m from shobot_yolo_detection
            if isinstance(detection_data, list):
                for det in detection_data:
                    depth_val = det.get("depth_m")
                    if depth_val is None:
                        continue
                    try:
                        depth_f = float(depth_val)
                    except (TypeError, ValueError):
                        continue
                    if depth_f < closest:
                        closest = depth_f

            # Backward-compatible format: {"Depth": "0.45m"}
            elif isinstance(detection_data, dict):
                camera_distance = detection_data.get("Depth", "No data")
                if camera_distance != "No data":
                    try:
                        closest = float(str(camera_distance).rstrip('m'))
                    except ValueError:
                        self.get_logger().warn(f"Bad camera depth value: {camera_distance}")

            if closest == float('inf'):
                self.camera_closest_distance = float('inf')
                # Avoid spamming if detections are empty.
                return

            self.camera_closest_distance = closest
            self.check_obstacle()
        except (ValueError, KeyError) as e:
            self.get_logger().error(f"Error parsing camera data: {e}")

    def image_callback(self, msg):
        """Run YOLO on RGB frames and publish detection info when objects appear."""
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Run YOLOv8 inference
            results = self.model(cv_image)  # Run YOLOv8 inference

            # Process detection results and publish to /detection_info
            for result in results:
                if result.boxes:  # Check if detection boxes exist
                    for box in result.boxes:
                        x1, y1, x2, y2 = box.xyxy
                        label = box.cls
                        self.get_logger().info(f"Object detected! Label: {label}, Bounding Box: {x1, y1, x2, y2}")
                        detection_info = json.dumps({"Depth": str(self.camera_closest_distance)})
                        self.detection_pub.publish(String(data=detection_info))
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")

    def depth_callback(self, msg):
        """Update closest camera depth from raw depth images."""
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            depth_array = np.array(cv_depth, dtype=np.float32)

            # Depth cameras often publish uint16 in millimeters; scale if needed.
            if depth_array.dtype == np.uint16:
                depth_array = depth_array * 0.001

            finite_vals = depth_array[np.isfinite(depth_array)]
            if finite_vals.size == 0:
                self.camera_closest_distance = float('inf')
                return

            min_depth = float(np.min(finite_vals))
            self.camera_closest_distance = min_depth
            detection_info = json.dumps({"Depth": f"{min_depth:.2f}m"})
            self.detection_pub.publish(String(data=detection_info))
            self.check_obstacle()
        except Exception as e:
            self.get_logger().error(f"Error in depth callback: {e}")

    def check_obstacle(self):
        """Stop motors if LiDAR or camera see an obstacle below thresholds."""
        if (self.lidar_closest_distance < LIDAR_THRESHOLD or
                self.camera_closest_distance < CAMERA_THRESHOLD):
            if not self.object_detected:
                self.get_logger().warn("Obstacle detected! Stopping motors.")
            self.object_detected = True
            self.stop_motors()
        else:
            if self.object_detected:
                self.get_logger().info("Obstacle cleared! Resuming movement.")
            self.object_detected = False
            self.resume_previous_speed()

    def cmd_vel_callback(self, msg):
        """Translate `/cmd_vel` into left/right RPM and send to motors."""
        linear_rpm = mps_to_rpm(msg.linear.x)
        angular_rpm = radps_to_rpm(msg.angular.z)

        left_motor_rpm = limit_speed(linear_rpm - angular_rpm)
        right_motor_rpm = limit_speed(-(linear_rpm + angular_rpm))

        self.previous_left_speed = int(left_motor_rpm)
        self.previous_right_speed = int(right_motor_rpm)

        if not self.object_detected:
            set_motor_speed('left', int(left_motor_rpm))
            set_motor_speed('right', int(right_motor_rpm))

            self.get_logger().info(f"Left Motor: {left_motor_rpm} RPM, Right Motor: {right_motor_rpm} RPM")

    def stop_motors(self):
        set_motor_speed('left', 0)
        set_motor_speed('right', 0)
        self.get_logger().info('Motors stopped')

    def resume_previous_speed(self):
        if self.previous_left_speed != 0 or self.previous_right_speed != 0:
            set_motor_speed('left', self.previous_left_speed)
            set_motor_speed('right', self.previous_right_speed)
        else:
            self.get_logger().warn("No previous speed to resume.")

    def run(self):
        rclpy.spin(self)

if __name__ == '__main__':
    rclpy.init()
    node = MotorControlNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        rclpy.shutdown()
