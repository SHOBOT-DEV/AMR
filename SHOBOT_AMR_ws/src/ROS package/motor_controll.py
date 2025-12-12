#!/usr/bin/env python3
"""Motor controller with obstacle handling, encoder publishing, and safety stops (ROS2)."""
import json
import serial
import struct
import time
from math import isnan

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, String


class MotorController(Node):
    """Translate `/cmd_vel` into motor RPM while fusing LiDAR/camera for safety."""

    def __init__(self):
        super().__init__('motor_control_node')

        self.ser = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=19200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.1,
        )

        self.WHEEL_RADIUS = 0.08
        self.WHEELBASE = 0.447967
        self.MAX_RPM = 3000
        self.speed_scale = 1.0  # multiplicative scale for all speeds
        self.SPEED_SCALE_MIN = 0.2
        self.SPEED_SCALE_MAX = 1.5
        self.SPEED_SCALE_STEP = 0.1
        self.LIDAR_THRESHOLD = 0.5
        self.CAMERA_THRESHOLD = 0.5

        self.previous_left_speed = 0
        self.previous_right_speed = 0
        self.object_detected = False
        self.lidar_closest_distance = float('inf')
        self.camera_closest_distance = float('inf')
        self.frame_counter = 0

        self.left_encoder_pub = self.create_publisher(Int32, '/left_wheel_encoder', 10)
        self.right_encoder_pub = self.create_publisher(Int32, '/right_wheel_encoder', 10)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(String, '/detection_info', self.camera_callback, 10)
        # Simple speed scaling control: publish 'i' to increase, 'd' to decrease
        self.create_subscription(String, '/motor_speed_scale', self.speed_scale_cb, 10)

        self.encoder_timer = self.create_timer(0.1, self.read_encoders)

        self.enable_motor()
        self.get_logger().info('Controller initialized with ROS2 teleop behavior')

    @staticmethod
    def crc16(data: bytes):
        """Compute CRC16 for motor driver frames."""
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

    def enable_motor(self):
        enable_command = b'\x01\x06\x10\x00\x00\x0A'
        crc_value = self.crc16(enable_command)
        crc_bytes = struct.pack('<H', crc_value)
        frame = enable_command + crc_bytes
        self.ser.write(frame)
        time.sleep(0.1)
        _ = self.ser.read(self.ser.in_waiting)

    def send_command(self, command: bytes):
        """Send a framed command to the motor driver and return its response."""
        crc_value = self.crc16(command)
        crc_bytes = struct.pack('<H', crc_value)
        frame = command + crc_bytes
        self.ser.write(frame)
        time.sleep(0.05)
        return self.ser.read(self.ser.in_waiting)

    def read_encoder(self, motor: str):
        """Read a signed 32-bit encoder count from the left or right wheel."""
        if motor == 'left':
            command = b'\x01\x03\x10\x0C\x00\x02'  # Left encoder (0x100C)
        else:
            command = b'\x01\x03\x10\x0E\x00\x02'  # Right encoder (0x100E)

        response = self.send_command(command)

        if response and len(response) >= 9:
            if struct.unpack('<H', response[-2:])[0] != self.crc16(response[:-2]):
                return None
            return struct.unpack('>i', response[3:7])[0]
        return None

    def read_encoders(self):
        """Continuously poll encoders and publish counts at 10 Hz."""
        left = self.read_encoder('left')
        right = self.read_encoder('right')

        if left is not None:
            self.left_encoder_pub.publish(Int32(data=left))
        if right is not None:
            self.right_encoder_pub.publish(Int32(data=right))

    def cmd_vel_callback(self, msg):
        """Convert `/cmd_vel` to wheel RPMs using original scaling and mix."""
        linear_rpm = (msg.linear.x * 60) / (2 * 3.14159 * self.WHEEL_RADIUS) * (1 / 0.0476)
        angular_rpm = (msg.angular.z * self.WHEELBASE * 60) / (2 * 3.14159 * self.WHEEL_RADIUS) * 10

        # Apply user speed scaling
        linear_rpm *= self.speed_scale
        angular_rpm *= self.speed_scale

        left_motor_rpm = linear_rpm - angular_rpm
        right_motor_rpm = -(linear_rpm + angular_rpm)

        left_motor_rpm = max(min(left_motor_rpm, self.MAX_RPM), -self.MAX_RPM)
        right_motor_rpm = max(min(right_motor_rpm, self.MAX_RPM), -self.MAX_RPM)

        self.previous_left_speed = int(left_motor_rpm)
        self.previous_right_speed = int(right_motor_rpm)

        if not self.object_detected:
            self.set_motor_speed('left', self.previous_left_speed)
            self.set_motor_speed('right', self.previous_right_speed)

    def speed_scale_cb(self, msg: String):
        """Adjust overall speed scale; expects 'i' to increase, 'd' to decrease."""
        data = msg.data.strip().lower()
        if data == 'i':
            self.speed_scale = min(self.speed_scale + self.SPEED_SCALE_STEP, self.SPEED_SCALE_MAX)
        elif data == 'd':
            self.speed_scale = max(self.speed_scale - self.SPEED_SCALE_STEP, self.SPEED_SCALE_MIN)
        else:
            return
        self.get_logger().info(f"Speed scale set to {self.speed_scale:.2f}")

    def set_motor_speed(self, motor: str, speed: int):
        """Write a target RPM to the left or right motor register."""
        speed_bytes = struct.pack('>i', speed)
        high_word = speed_bytes[:2]
        low_word = speed_bytes[2:]
        corrected_speed_bytes = low_word + high_word

        if motor == 'left':
            command = b'\x01\x10\x10\x04\x00\x02\x04' + corrected_speed_bytes
        else:
            command = b'\x01\x10\x10\x06\x00\x02\x04' + corrected_speed_bytes

        self.send_command(command)

    def lidar_callback(self, msg):
        """Track closest LiDAR obstacle in the front arc and update state."""
        self.frame_counter += 1
        if self.frame_counter % 5 != 0:
            return
        self.frame_counter = 0

        num_readings = len(msg.ranges)
        half_range = num_readings // 2
        shift = num_readings // 4
        rotated_ranges = np.roll(np.array(msg.ranges), shift)
        front_180_deg_data = rotated_ranges[:half_range]
        self.lidar_closest_distance = float('inf')

        for distance in front_180_deg_data:
            if isnan(distance):
                continue
            if distance < self.lidar_closest_distance:
                self.lidar_closest_distance = distance

        self.check_obstacle()

    def camera_callback(self, msg):
        """React to camera-based obstacle distances and slow/stop when needed."""
        try:
            if not msg.data:
                self.camera_closest_distance = float('inf')
                return

            detection_data = json.loads(msg.data)
            camera_distance = detection_data.get('Depth', 'No data')
            if camera_distance == 'No data':
                self.camera_closest_distance = float('inf')
            else:
                self.camera_closest_distance = float(camera_distance.rstrip('m'))

            stop_threshold = 0.2
            slow_threshold = self.CAMERA_THRESHOLD

            if self.camera_closest_distance < stop_threshold:
                self.get_logger().warn('Obstacle detected very close by camera! Stopping motors.')
                self.object_detected = True
                self.stop_motors()
            elif stop_threshold <= self.camera_closest_distance < slow_threshold:
                self.get_logger().warn('Obstacle nearby detected by camera! Slowing down to 50%.')
                self.object_detected = True
                self.slow_down_motors()
            else:
                self.object_detected = False
                self.check_obstacle()
        except (ValueError, KeyError, json.JSONDecodeError) as e:
            self.get_logger().error(f"Error parsing camera data: {e}")

    def check_obstacle(self):
        """Select stop/slow/resume actions based on LiDAR and camera ranges."""
        stop_threshold = 0.3
        slow_threshold = 0.6

        if self.lidar_closest_distance < stop_threshold or (
                self.object_detected and self.camera_closest_distance < stop_threshold):
            self.get_logger().warn('Obstacle detected very close! Stopping motors.')
            self.stop_motors()
        elif stop_threshold <= self.lidar_closest_distance < self.LIDAR_THRESHOLD or (
                self.object_detected and stop_threshold <= self.camera_closest_distance < self.CAMERA_THRESHOLD):
            self.get_logger().warn('Obstacle nearby! Slowing down to 50%.')
            self.slow_down_motors()
        else:
            self.resume_previous_speed()

    def slow_down_motors(self):
        """Half the previous RPMs to slow down without stopping."""
        reduced_left_speed = self.previous_left_speed // 2
        reduced_right_speed = self.previous_right_speed // 2
        self.set_motor_speed('left', reduced_left_speed)
        self.set_motor_speed('right', reduced_right_speed)

    def stop_motors(self):
        """Command both motors to zero RPM."""
        self.set_motor_speed('left', 0)
        self.set_motor_speed('right', 0)
        self.get_logger().info('Motors stopped')

    def resume_previous_speed(self):
        if not self.object_detected and (self.previous_left_speed != 0 or self.previous_right_speed != 0):
            self.set_motor_speed('left', self.previous_left_speed)
            self.set_motor_speed('right', self.previous_right_speed)

    def shutdown(self):
        self.stop_motors()
        if self.ser.is_open:
            self.ser.close()
        self.get_logger().info('Shutdown complete')


def main():
    rclpy.init()
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
