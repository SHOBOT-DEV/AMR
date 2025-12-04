#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class YoloDetectionNode(Node):
    """Placeholder YOLO detection node publishing dummy detections."""

    def __init__(self):
        super().__init__("shobot_yolo_detection")
        self.declare_parameter("detection_topic", "/yolo/detections")
        topic = self.get_parameter("detection_topic").value
        self.publisher = self.create_publisher(String, topic, 10)
        self.timer = self.create_timer(1.0, self.publish_dummy)
        self.get_logger().info(f"Publishing dummy detections on {topic}")

    def publish_dummy(self):
        msg = String()
        msg.data = "[]"  # JSON array placeholder
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
