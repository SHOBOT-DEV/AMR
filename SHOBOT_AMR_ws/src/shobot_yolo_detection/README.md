# shobot_yolo_detection

YOLOv10 detector node for object detection with optional annotated image output.

## Launch
- `ros2 launch shobot_yolo_detection shobot_yolo_detection_launch.py`

How to use:
1) Place your YOLO model file (e.g., `yolov10n.pt`) where the node can read it.
2) Launch the detector (example):
```bash
ros2 launch shobot_yolo_detection shobot_yolo_detection_launch.py model_path:=yolov10n.pt detection_topic:=/detection_info annotated_topic:=/yolo/detection_image score_threshold:=0.25 use_ros_camera:=False
```
3) Subscribe to the detection topic to receive detections; subscribe to the annotated image topic to see bounding boxes overlaid.

Key params:
- `model_path`: YOLO model file
- `detection_topic`: publishes detections (e.g., JSON/custom msg)
- `annotated_topic`: publishes annotated image (optional)
- `use_ros_camera`: read from ROS camera topics vs. direct device
- `score_threshold`: detection confidence cutoff
- `frame_skip`: process every Nth frame to save compute
- `color_topic` / `depth_topic`: image topics when `use_ros_camera` is true
- `max_detections`: cap number of detections per frame (0 = unlimited)
