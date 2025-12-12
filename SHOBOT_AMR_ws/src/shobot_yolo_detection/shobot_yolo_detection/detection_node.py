#!/usr/bin/env python3
"""
YOLOv10 Detection Node
===================================================
Subscribes to color + depth images and publishes:
1. JSON detection info (/detection_info)
2. Annotated detection image (/detection_image)
"""

import json
import contextlib
import io

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class DetectionNode(Node):
    def __init__(self):
        super().__init__("yolo_detection_node")

        self.declare_parameter("model_path", "yolov10n.pt")
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("detection_topic", "/detection_info")
        self.declare_parameter("annotated_topic", "/detection_image")
        self.declare_parameter("score_threshold", 0.25)

        self.model_path = self.get_parameter("model_path").value
        self.color_topic = self.get_parameter("color_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.score_threshold = float(self.get_parameter("score_threshold").value)

        self.bridge = CvBridge()

        self.color_img = None
        self.depth_img = None

        self.create_subscription(Image, self.color_topic, self.color_cb, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)

        self.det_pub = self.create_publisher(String, "/detection_info", 10)
        self.img_pub = self.create_publisher(Image, "/detection_image", 10)

        # Load YOLO model
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = YOLO(self.model_path)
        self.model.to(self.device)

        self.timer = self.create_timer(0.05, self.run_detection)

        self.get_logger().info("YOLO Detection Node started.")

    def color_cb(self, msg):
        self.color_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_cb(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def run_detection(self):
        if self.color_img is None or self.depth_img is None:
            return

        with contextlib.redirect_stdout(io.StringIO()):
            results = self.model(self.color_img)

        detections = []
        annotated = self.color_img.copy()
        h, w = self.depth_img.shape[:2]

        for box in results[0].boxes:
            conf = float(box.conf)
            if conf < self.score_threshold:
                continue

            cls_id = int(box.cls)
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

            cx = max(0, min((x1 + x2) // 2, w - 1))
            cy = max(0, min((y1 + y2) // 2, h - 1))

            depth_val = float(self.depth_img[cy, cx]) * 0.001

            detections.append({
                "label": results[0].names[cls_id],
                "confidence": conf,
                "bbox": [x1, y1, x2, y2],
                "centroid_px": [cx, cy],
                "depth_m": depth_val
            })

            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.putText(annotated, f"{results[0].names[cls_id]} {conf:.2f}",
                        (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0,255,0), 2)

        self.det_pub.publish(String(data=json.dumps(detections)))
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(annotated, "bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
