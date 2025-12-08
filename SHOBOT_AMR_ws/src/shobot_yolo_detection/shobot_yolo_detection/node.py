#!/usr/bin/env python3
"""YOLOv10 RealSense detector publishing JSON detections and annotated images."""
import contextlib
import io
import json
from typing import List

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class DepthDetector(Node):
    """Capture RealSense frames, run YOLOv10, and publish detections + image."""

    def __init__(self):
        super().__init__("depth_detector")
        self.declare_parameter("model_path", "yolov10n.pt")
        self.declare_parameter("detection_topic", "/detection_info")
        self.declare_parameter("annotated_topic", "detection_image")
        self.declare_parameter("frame_skip", 5)
        self.declare_parameter("score_threshold", 0.25)
        self.declare_parameter("use_display", False)
        self.declare_parameter("use_ros_camera", False)
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")

        model_path = self.get_parameter("model_path").value
        detection_topic = self.get_parameter("detection_topic").value
        annotated_topic = self.get_parameter("annotated_topic").value
        self.frame_skip = int(self.get_parameter("frame_skip").value)
        self.score_threshold = float(self.get_parameter("score_threshold").value)
        self.use_display = bool(self.get_parameter("use_display").value)
        self.use_ros_camera = bool(self.get_parameter("use_ros_camera").value)
        self.color_topic = self.get_parameter("color_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value

        self.det_pub = self.create_publisher(String, detection_topic, 10)
        self.image_pub = self.create_publisher(Image, annotated_topic, 10)
        self.bridge = CvBridge()

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {self.device}")

        self.model = YOLO(model_path, verbose=False)
        self.model.to(self.device)

        self.frame_count = 0
        self.latest_depth = None
        self.latest_color = None

        if self.use_ros_camera:
            self.get_logger().info(f"Using ROS topics: color={self.color_topic}, depth={self.depth_topic}")
            self.create_subscription(Image, self.color_topic, self.color_cb, 10)
            self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)
            self.timer = self.create_timer(0.03, self.tick_ros)
        else:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.pipeline.start(config)
            self.align = rs.align(rs.stream.color)
            self.timer = self.create_timer(0.03, self.tick_rs)

        self.get_logger().info(
            f"YOLOv10 running. Pub detections: {detection_topic}, annotated image: {annotated_topic}, model: {model_path}"
        )

    def tick_rs(self):
        self.frame_count += 1
        if self.frame_count % self.frame_skip != 0:
            return

        depth_frame, color_frame = self.capture_frames()
        if depth_frame is None or color_frame is None:
            return

        color_image, detections, annotated = self.process(depth_frame, color_frame)
        self.publish_results(detections, annotated)
        if self.use_display:
            cv2.imshow("YOLOv10 Object Detection with Depth", annotated)
            cv2.waitKey(1)

    def tick_ros(self):
        self.frame_count += 1
        if self.frame_count % self.frame_skip != 0:
            return
        if self.latest_depth is None or self.latest_color is None:
            return
        depth_frame = self.latest_depth
        color_image = self.latest_color
        with contextlib.redirect_stdout(io.StringIO()):
            results = self.model(color_image)
        detections = self._serialize_detections_from_arrays(results, depth_frame)
        annotated = self._annotate_from_arrays(color_image, depth_frame, results)
        self.publish_results(detections, annotated)
        if self.use_display:
            cv2.imshow("YOLOv10 Object Detection with Depth", annotated)
            cv2.waitKey(1)

    def capture_frames(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            return None, None
        return depth_frame, color_frame

    def process(self, depth_frame, color_frame):
        color_image = np.asanyarray(color_frame.get_data())

        with contextlib.redirect_stdout(io.StringIO()):
            results = self.model(color_image)

        detections = self._serialize_detections(results, depth_frame)
        annotated = self._annotate(color_image, depth_frame, results)
        return color_image, detections, annotated

    def _serialize_detections(self, results, depth_frame) -> List[dict]:
        detections = []
        if not results or len(results[0].boxes) == 0:
            return detections

        names = results[0].names
        depth_width = depth_frame.get_width()
        depth_height = depth_frame.get_height()

        for box in results[0].boxes:
            conf = float(box.conf.item())
            if conf < self.score_threshold:
                continue
            cls_idx = int(box.cls.item())
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            cx = max(0, min((x1 + x2) // 2, depth_width - 1))
            cy = max(0, min((y1 + y2) // 2, depth_height - 1))
            depth_value = depth_frame.get_distance(cx, cy)
            detections.append(
                {
                    "label": names.get(cls_idx, str(cls_idx)),
                    "confidence": conf,
                    "bbox": [x1, y1, x2, y2],
                    "centroid_px": [cx, cy],
                    "depth_m": depth_value,
                }
            )
        return detections

    def _serialize_detections_from_arrays(self, results, depth_image) -> List[dict]:
        detections = []
        if not results or len(results[0].boxes) == 0:
            return detections
        names = results[0].names
        h, w = depth_image.shape[:2]
        depth_scale = 0.001 if depth_image.dtype == np.uint16 else 1.0
        for box in results[0].boxes:
            conf = float(box.conf.item())
            if conf < self.score_threshold:
                continue
            cls_idx = int(box.cls.item())
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            cx = max(0, min((x1 + x2) // 2, w - 1))
            cy = max(0, min((y1 + y2) // 2, h - 1))
            depth_value = float(depth_image[cy, cx]) * depth_scale
            detections.append(
                {
                    "label": names.get(cls_idx, str(cls_idx)),
                    "confidence": conf,
                    "bbox": [x1, y1, x2, y2],
                    "centroid_px": [cx, cy],
                    "depth_m": depth_value,
                }
            )
        return detections

    def _annotate(self, image, depth_frame, results):
        annotated = image.copy()
        if not results or len(results[0].boxes) == 0:
            return annotated

        names = results[0].names
        depth_width = depth_frame.get_width()
        depth_height = depth_frame.get_height()

        for box in results[0].boxes:
            conf = float(box.conf.item())
            if conf < self.score_threshold:
                continue
            cls_idx = int(box.cls.item())
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            cx = max(0, min((x1 + x2) // 2, depth_width - 1))
            cy = max(0, min((y1 + y2) // 2, depth_height - 1))
            depth_value = depth_frame.get_distance(cx, cy)

            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(annotated, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(
                annotated,
                f"{names.get(cls_idx, cls_idx)} {conf:.2f} | {depth_value:.2f}m",
                (x1, max(0, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
            )
        return annotated

    def _annotate_from_arrays(self, image, depth_image, results):
        annotated = image.copy()
        if not results or len(results[0].boxes) == 0:
            return annotated
        names = results[0].names
        h, w = depth_image.shape[:2]
        depth_scale = 0.001 if depth_image.dtype == np.uint16 else 1.0
        for box in results[0].boxes:
            conf = float(box.conf.item())
            if conf < self.score_threshold:
                continue
            cls_idx = int(box.cls.item())
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            cx = max(0, min((x1 + x2) // 2, w - 1))
            cy = max(0, min((y1 + y2) // 2, h - 1))
            depth_value = float(depth_image[cy, cx]) * depth_scale
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(annotated, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(
                annotated,
                f"{names.get(cls_idx, cls_idx)} {conf:.2f} | {depth_value:.2f}m",
                (x1, max(0, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
            )
        return annotated

    def publish_results(self, detections, annotated_image):
        self.det_pub.publish(String(data=json.dumps(detections)))
        ros_image = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
        self.image_pub.publish(ros_image)

    def destroy_node(self):
        try:
            if not self.use_ros_camera and hasattr(self, "pipeline"):
                self.pipeline.stop()
            cv2.destroyAllWindows()
        finally:
            super().destroy_node()

    def color_cb(self, msg: Image):
        try:
            self.latest_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert color image: {e}")

    def depth_cb(self, msg: Image):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.latest_depth = cv_depth
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DepthDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
