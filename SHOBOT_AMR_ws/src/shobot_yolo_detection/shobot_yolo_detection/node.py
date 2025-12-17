#!/usr/bin/env python3
"""Camera publisher and YOLO detection nodes for SHOBOT AMR."""

from __future__ import annotations

import copy
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)
import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
import torch
import pyrealsense2 as rs
from ultralytics import YOLO


def _default_model_path() -> Path:
    """Locate a bundled YOLO weights file (either installed share dir or source tree)."""
    candidates: list[Path] = []

    try:
        share_dir = Path(get_package_share_directory("shobot_yolo_detection"))
        weights_dir = share_dir / "weights"
        if weights_dir.exists():
            candidates.extend(sorted(weights_dir.glob("*.pt")))
    except PackageNotFoundError:
        pass

    source_weights = Path(__file__).resolve().parents[2] / "weights"
    if source_weights.exists():
        candidates.extend(sorted(source_weights.glob("*.pt")))

    if candidates:
        for preferred in ("yolov10n.pt", "yolov10s.pt"):
            for candidate in candidates:
                if candidate.name == preferred:
                    return candidate
        return candidates[0]

    fallback = Path(__file__).resolve().parents[4] / "yolov10n.pt"
    return fallback


# Shared ---------------------------------------------------------------------
@dataclass
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    coeffs: list[float]

    def to_camera_info(self, frame_id: str) -> CameraInfo:
        """Convert intrinsics into a sensor_msgs/CameraInfo message."""
        msg = CameraInfo()
        msg.header.frame_id = frame_id
        msg.width = int(self.width)
        msg.height = int(self.height)

        # K (3x3) row-major
        msg.K = [float(self.fx), 0.0, float(self.cx), 0.0, float(self.fy), float(self.cy), 0.0, 0.0, 1.0]
        # P (3x4) projection matrix
        msg.P = [
            float(self.fx), 0.0, float(self.cx), 0.0,
            0.0, float(self.fy), float(self.cy), 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        # R (3x3) identity
        msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # Distortion coefficients
        msg.D = list(self.coeffs) if self.coeffs is not None else []
        msg.distortion_model = "plumb_bob"
        return msg


# RealSense camera publisher --------------------------------------------------
class RealSenseCameraNode(Node):
    """Publishes aligned RealSense color/depth images and camera info."""

    def __init__(self) -> None:
        super().__init__("shobot_realsense_camera")

        self.bridge = CvBridge()
        self.fps = float(self.declare_parameter("frame_rate", 15.0).value)
        self.width = int(self.declare_parameter("width", 640).value)
        self.height = int(self.declare_parameter("height", 480).value)
        self.color_topic = self.declare_parameter("color_topic", "/camera/color/image_raw").value
        self.depth_topic = self.declare_parameter("depth_topic", "/camera/depth/image_raw").value
        self.color_info_topic = self.declare_parameter(
            "color_info_topic", "/camera/color/camera_info"
        ).value
        self.depth_info_topic = self.declare_parameter(
            "depth_info_topic", "/camera/depth/camera_info"
        ).value
        self.color_frame_id = self.declare_parameter("color_frame_id", "camera_color_optical_frame").value
        self.depth_frame_id = self.declare_parameter("depth_frame_id", "camera_depth_optical_frame").value

        self.color_pub = self.create_publisher(Image, self.color_topic, 10)
        self.depth_pub = self.create_publisher(Image, self.depth_topic, 10)
        self.color_info_pub = self.create_publisher(CameraInfo, self.color_info_topic, 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, self.depth_info_topic, 10)

        # RealSense pipeline & streams
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, int(self.fps))
        cfg.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, int(self.fps))

        try:
            profile = self.pipeline.start(cfg)
        except Exception as exc:  # hardware / driver start failure
            self.get_logger().fatal(f"Failed to start RealSense pipeline: {exc}")
            raise

        self.align = rs.align(rs.stream.color)
        self.depth_scale = float(profile.get_device().first_depth_sensor().get_depth_scale())

        # build Intrinsics from the streaming profiles
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        self.color_intr = self._intrinsics_from_profile(color_profile)
        self.depth_intr = self._intrinsics_from_profile(depth_profile)

        # publish timer
        self.timer = self.create_timer(1.0 / self.fps, self._publish_frame)
        self.get_logger().info(
            "RealSense camera started — publishing color/depth images for other nodes and UI."
        )

    @staticmethod
    def _intrinsics_from_profile(profile: rs.video_stream_profile) -> Intrinsics:
        intr = profile.get_intrinsics()
        return Intrinsics(
            width=int(intr.width),
            height=int(intr.height),
            fx=float(intr.fx),
            fy=float(intr.fy),
            cx=float(intr.ppx),
            cy=float(intr.ppy),
            coeffs=list(intr.coeffs) if intr.coeffs is not None else [],
        )

    def _publish_frame(self) -> None:
        """Capture frames, align, publish images and CameraInfo."""
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=5000)
            aligned = self.align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()

            if not color_frame or not depth_frame:
                self.get_logger().warning("No frames received from RealSense (skipping).")
                return

            stamp = self.get_clock().now().to_msg()

            color_image = np.asanyarray(color_frame.get_data())
            # depth raw in uint16 (z16) -> convert to meters float32
            depth_raw = np.asanyarray(depth_frame.get_data())
            depth_meters = depth_raw.astype(np.float32) * float(self.depth_scale)

            # Publish color image (bgr8)
            color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            color_msg.header.stamp = stamp
            color_msg.header.frame_id = self.color_frame_id
            self.color_pub.publish(color_msg)

            # Publish depth image as 32FC1 (meters)
            depth_msg = self.bridge.cv2_to_imgmsg(depth_meters, encoding="32FC1")
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = self.depth_frame_id
            self.depth_pub.publish(depth_msg)

            # Publish camera infos
            color_info = copy.deepcopy(self.color_intr.to_camera_info(self.color_frame_id))
            color_info.header.stamp = stamp
            self.color_info_pub.publish(color_info)

            depth_info = copy.deepcopy(self.depth_intr.to_camera_info(self.depth_frame_id))
            depth_info.header.stamp = stamp
            self.depth_info_pub.publish(depth_info)
        except Exception as exc:  # pragma: no cover - hardware failures
            self.get_logger().error(f"RealSense publish error: {exc}")

    def destroy_node(self) -> None:
        """Stop pipeline then call base destroy."""
        try:
            self.pipeline.stop()
        except Exception:
            pass
        super().destroy_node()


# YOLO detector ---------------------------------------------------------------
class YoloDetectionNode(Node):
    """Runs YOLO on incoming camera frames and publishes detections + annotated image."""

    def __init__(self) -> None:
        super().__init__("shobot_yolo_detector")

        self.bridge = CvBridge()
        default_model = _default_model_path()

        self.model_path = Path(
            self.declare_parameter("model_path", str(default_model)).value
        ).expanduser()
        self.confidence = float(self.declare_parameter("confidence_threshold", 0.4).value)
        self.color_topic = self.declare_parameter("color_topic", "/camera/color/image_raw").value
        self.depth_topic = self.declare_parameter("depth_topic", "/camera/depth/image_raw").value
        self.camera_info_topic = self.declare_parameter("camera_info_topic", "/camera/color/camera_info").value

        self.det_pub = self.create_publisher(String, "/detection_info", 10)
        self.image_pub = self.create_publisher(Image, "/detection_image", 10)

        # CameraInfo will arrive via subscription — needed to compute 3D coords
        self.camera_info: Optional[CameraInfo] = None
        self.warned_no_info = False
        self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_cb, 10)

        # Load YOLO
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Loading YOLO model {self.model_path} on {self.device}")
        try:
            self.model = YOLO(str(self.model_path))
            self.model.to(self.device)
        except Exception as exc:
            self.get_logger().fatal(f"Failed to load YOLO model: {exc}")
            raise

        # warm-up using hint sizes (will use defaults until CameraInfo arrives)
        try:
            _ = self.model(np.zeros((self.height_hint, self.width_hint, 3), dtype=np.uint8), verbose=False)
        except Exception:
            # warmup is best-effort
            pass

        # Subscribers (synchronized)
        self.color_sub = message_filters.Subscriber(self, Image, self.color_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)

        # ApproximateTimeSynchronizer signature: (subscribers, queue_size, slop)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], 10, 0.1
        )
        self.sync.registerCallback(self._synced_callback)

        # GUI for quick debug (optional)
        cv2.startWindowThread()
        cv2.namedWindow("YOLO Detector", cv2.WINDOW_NORMAL)

        self.get_logger().info("YOLO detection node ready and waiting for camera frames.")

    @property
    def width_hint(self) -> int:
        if self.camera_info:
            return int(self.camera_info.width)
        return 640

    @property
    def height_hint(self) -> int:
        if self.camera_info:
            return int(self.camera_info.height)
        return 480

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def _synced_callback(self, color_msg: Image, depth_msg: Image) -> None:
        """Callback where detection happens. Uses camera_info to compute 3D coords."""
        if self.camera_info is None:
            if not self.warned_no_info:
                self.get_logger().warn("Waiting for CameraInfo before running detection")
                self.warned_no_info = True
            return
        # reset warning once we have camera info
        self.warned_no_info = False

        try:
            color_img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth_array = self.bridge.imgmsg_to_cv2(depth_msg)  # could be float32 meters or uint16 raw
            # If the depth message is 16UC1 (raw mm), convert to meters. If already 32FC1 assume meters.
            if hasattr(depth_msg, "encoding") and depth_msg.encoding == "16UC1":
                depth_array = depth_array.astype(np.float32) * 0.001
            else:
                depth_array = depth_array.astype(np.float32)
        except Exception as exc:
            self.get_logger().error(f"Failed to convert incoming images: {exc}")
            return

        # Run YOLO inference
        try:
            with torch.no_grad():
                result = self.model(color_img, verbose=False)[0]
        except Exception as exc:
            self.get_logger().error(f"YOLO inference failure: {exc}")
            return

        annotated = color_img.copy()
        names = getattr(result, "names", {})

        # iterate detections
        if result.boxes is not None and len(result.boxes) > 0:
            boxes_np = result.boxes.xyxy.cpu().numpy()
            confs = result.boxes.conf.cpu().numpy()
            cls_idxs = result.boxes.cls.cpu().numpy().astype(int)

            for box, conf, cls_idx in zip(boxes_np, confs, cls_idxs):
                if conf < self.confidence:
                    continue

                x1, y1, x2, y2 = box.astype(int)
                # centroid
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                if cy < 0 or cy >= depth_array.shape[0] or cx < 0 or cx >= depth_array.shape[1]:
                    # centroid outside depth map -> skip position calc but still draw bbox
                    depth_value = float("nan")
                else:
                    depth_value = float(depth_array[cy, cx])

                position = None
                if np.isfinite(depth_value) and depth_value > 0.0:
                    fx = float(self.camera_info.K[0])
                    fy = float(self.camera_info.K[4])
                    cx0 = float(self.camera_info.K[2])
                    cy0 = float(self.camera_info.K[5])
                    X = (cx - cx0) * depth_value / fx
                    Y = (cy - cy0) * depth_value / fy
                    position = {"X": float(X), "Y": float(Y), "Z": float(depth_value)}

                class_name = names.get(cls_idx, str(cls_idx))
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{class_name} {conf:.2f}"
                if position:
                    label += f" Z={position['Z']:.2f}m"
                cv2.putText(annotated, label, (x1, max(20, y1 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                detection = {
                    "class": class_name,
                    "confidence": float(conf),
                    "bbox": [int(x1), int(y1), int(x2), int(y2)],
                    "centroid_px": [int(cx), int(cy)],
                }
                if position:
                    detection["3d_position_m"] = position
                # publish JSON detection message
                self.det_pub.publish(String(data=json.dumps(detection)))

        # Publish annotated frame with same header (so UI timestamps align)
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            annotated_msg.header = color_msg.header
            self.image_pub.publish(annotated_msg)
        except Exception as exc:
            self.get_logger().error(f"Failed to publish annotated image: {exc}")

        # Optional local display for debugging (will not run in headless environments)
        try:
            cv2.imshow("YOLO Detector", annotated)
            cv2.waitKey(1)
        except Exception:
            # ignore display errors in headless/no-GUI systems
            pass

    def destroy_node(self) -> None:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


# Entrypoints ----------------------------------------------------------------
def run_camera(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RealSenseCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def run_detector(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(args: Optional[list[str]] = None) -> None:
    # default: run detector. Use run_camera() if you want the camera node instead.
    run_detector(args=args)


if __name__ == "__main__":
    main()
