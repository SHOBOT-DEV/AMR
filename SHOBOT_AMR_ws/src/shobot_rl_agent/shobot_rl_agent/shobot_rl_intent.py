#!/usr/bin/env python3
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Imu
from vision_msgs.msg import Detection2DArray


class IntentPredictNode(Node):
    """
    Predicts navigation intent:
    proceed / slow_down / yield / reroute
    """

    def __init__(self):
        super().__init__("shobot_rl_intent")

        self.declare_parameter("model_path", "")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("bbox_topic", "/detections")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("decision_topic", "/rl_intent/decision")

        self.latest_scan: Optional[LaserScan] = None
        self.latest_bbox: Optional[Detection2DArray] = None
        self.latest_imu: Optional[Imu] = None

        self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self._scan_cb,
            10,
        )
        self.create_subscription(
            Detection2DArray,
            self.get_parameter("bbox_topic").value,
            self._bbox_cb,
            5,
        )
        self.create_subscription(
            Imu,
            self.get_parameter("imu_topic").value,
            self._imu_cb,
            10,
        )

        self.decision_pub = self.create_publisher(
            String,
            self.get_parameter("decision_topic").value,
            10,
        )

        self.create_timer(0.2, self._tick)  # 5 Hz

        self.get_logger().info("RL Intent Prediction node started")

    # --------------------------------------------------
    def _scan_cb(self, msg):
        self.latest_scan = msg

    def _bbox_cb(self, msg):
        self.latest_bbox = msg

    def _imu_cb(self, msg):
        self.latest_imu = msg

    # --------------------------------------------------
    def _tick(self):
        if not self.latest_scan:
            return

        decision = self._heuristic_decision()
        self.decision_pub.publish(String(data=decision))

    def _heuristic_decision(self) -> str:
        min_r = min(self.latest_scan.ranges) if self.latest_scan.ranges else 5.0
        humans = len(self.latest_bbox.detections) if self.latest_bbox else 0

        if min_r < 0.7 and humans >= 2:
            return "yield"
        if humans >= 3:
            return "slow_down"
        if min_r < 0.6:
            return "reroute"
        return "proceed"


def main():
    rclpy.init()
    node = IntentPredictNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
