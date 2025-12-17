#!/usr/bin/env python3
"""Teleop motor controller that fuses LiDAR/camera stops with `/cmd_vel` inputs (ROS2)."""
import json

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from motor_utils import enable_motor, limit_speed, mps_to_rpm, radps_to_rpm, set_motor_speed

# Constants for thresholds
LIDAR_THRESHOLD = 0.5  # Meters
CAMERA_THRESHOLD = 0.5  # Meters
PUBLISH_RATE = 2  # Hz, to limit computation and data flow


class MotorControlNode(Node):
    """Convert velocity commands to wheel RPMs while enforcing obstacle stops."""

    def __init__(self):
        super().__init__('motor_control_node')

        self.previous_left_speed = 0
        self.previous_right_speed = 0
        self.object_detected = False
        self.lidar_closest_distance = float('inf')
        self.camera_closest_distance = float('inf')
        self.resume_timer = None

        self.motor_status_pub = self.create_publisher(String, '/motor_status', 10)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(String, '/detection_info', self.camera_callback, 10)

        enable_motor()
        self.get_logger().info('Motor control node initialized (ROS2).')

        self.create_timer(1.0 / PUBLISH_RATE, self.publish_status)

    def lidar_callback(self, msg):
        """Track closest LiDAR range and trigger obstacle logic."""
        self.lidar_closest_distance = min(msg.ranges)
        self.check_obstacle()

    def camera_callback(self, msg):
        """Parse YOLO depth data and update closest camera distance."""
        try:
            detection_data = json.loads(msg.data)
            if detection_data and 'Depth' in detection_data:
                depth_value = detection_data['Depth'].rstrip('m')
                if depth_value != 'No data':
                    self.camera_closest_distance = float(depth_value)
                else:
                    self.camera_closest_distance = float('inf')
            else:
                self.camera_closest_distance = float('inf')
            self.check_obstacle()
        except (ValueError, KeyError, json.JSONDecodeError) as e:
            self.get_logger().error(f"Error parsing camera data: {e}")
            self.camera_closest_distance = float('inf')
            self.check_obstacle()

    def cmd_vel_callback(self, msg):
        """Translate `/cmd_vel` to left/right RPM and send to motors."""
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

    def check_obstacle(self):
        """Stop or resume depending on LiDAR/camera thresholds."""
        self.get_logger().info(f"Lidar Distance: {self.lidar_closest_distance}, Threshold: {LIDAR_THRESHOLD}")
        self.get_logger().info(f"Camera Distance: {self.camera_closest_distance}, Threshold: {CAMERA_THRESHOLD}")

        if (self.lidar_closest_distance < LIDAR_THRESHOLD or
                self.camera_closest_distance < CAMERA_THRESHOLD):
            if not self.object_detected:
                self.get_logger().warn('Obstacle detected! Stopping motors.')
            self.object_detected = True
            self.stop_motors()
        else:
            if self.object_detected:
                self.get_logger().info('Obstacle cleared! Resuming movement after delay.')
            self.object_detected = False
            self.resume_previous_speed_with_delay()

    def stop_motors(self):
        """Immediately stop both motors."""
        set_motor_speed('left', 0)
        set_motor_speed('right', 0)
        self.get_logger().info('Motors stopped')

    def resume_previous_speed_with_delay(self):
        """Resume prior RPMs after a short delay if available."""
        if self.previous_left_speed != 0 or self.previous_right_speed != 0:
            if self.resume_timer:
                self.resume_timer.cancel()
            self.resume_timer = self.create_timer(2.0, self._resume_once)
        else:
            self.get_logger().warn('No previous speed to resume.')

    def _resume_once(self):
        # One-shot timer helper
        set_motor_speed('left', self.previous_left_speed)
        set_motor_speed('right', self.previous_right_speed)
        self.get_logger().info(
            f"Resumed previous speed - Left Motor: {self.previous_left_speed} RPM, Right Motor: {self.previous_right_speed} RPM")
        if self.resume_timer:
            self.resume_timer.cancel()
            self.resume_timer = None

    def publish_status(self):
        """Publish motor status at a limited rate until shutdown."""
        self.motor_status_pub.publish(
            String(data=f"Motor running: Left={self.previous_left_speed}, Right={self.previous_right_speed}"))


def main():
    rclpy.init()
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
