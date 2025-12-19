import math
import random
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image, Imu
from vision_msgs.msg import Detection2DArray


class RLAgentNode(Node):
    """
    Minimal RL policy runner.
    - Subscribes to LaserScan + Odometry for observations.
    - Uses a Torch policy if provided, otherwise falls back to a safe heuristic.
    - Publishes Twist commands.
    """

    def __init__(self):
        super().__init__("shobot_rl_agent")
        self.declare_parameter("model_path", "")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_speed", 0.2)
        self.declare_parameter("angular_speed", 0.4)

        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        cmd_vel_topic = (
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        )
        self.linear_speed = (
            self.get_parameter("linear_speed").get_parameter_value().double_value
        )
        self.angular_speed = (
            self.get_parameter("angular_speed").get_parameter_value().double_value
        )

        self.policy = self._load_policy(model_path)
        self.latest_scan: Optional[LaserScan] = None
        self.latest_odom: Optional[Odometry] = None

        self.create_subscription(LaserScan, scan_topic, self._scan_callback, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_timer(0.1, self._tick)  # 10 Hz loop for action publishing

        self.get_logger().info(
            f"RL agent started. Model: {model_path if model_path else 'heuristic'}, "
            f"scan: {scan_topic}, odom: {odom_topic}, cmd_vel: {cmd_vel_topic}"
        )

    def _load_policy(self, model_path: str):
        if not model_path:
            self.get_logger().warn("No model_path provided, using heuristic policy.")
            return None

        path = Path(model_path)
        if not path.exists():
            self.get_logger().warn(
                f"model_path '{model_path}' not found, falling back to heuristic."
            )
            return None

        try:
            import torch  # type: ignore
        except ImportError:
            self.get_logger().warn(
                "Torch not available in environment; cannot load model. "
                "Install torch or leave model_path empty for heuristic."
            )
            return None

        try:
            policy = torch.jit.load(str(path))
            policy.eval()
            self.get_logger().info(f"Loaded TorchScript policy from {path}")
            return policy
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to load model '{path}': {exc}")
            return None

    def _scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def _odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def _tick(self):
        if self.latest_scan is None or self.latest_odom is None:
            return

        linear, angular = self._choose_action(self.latest_scan, self.latest_odom)
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

    def _choose_action(
        self, scan: LaserScan, odom: Odometry
    ) -> Tuple[float, float]:
        # Model path provided and torch policy loaded
        if self.policy is not None:
            try:
                import torch  # type: ignore

                scan_tensor = torch.tensor(scan.ranges, dtype=torch.float32)
                # Simple state: scan + linear, angular velocity
                lin = odom.twist.twist.linear.x
                ang = odom.twist.twist.angular.z
                state = torch.cat(
                    [
                        scan_tensor,
                        torch.tensor([lin, ang], dtype=torch.float32),
                    ]
                )
                with torch.no_grad():
                    action = self.policy(state.unsqueeze(0)).squeeze(0)
                linear = float(action[0].item())
                angular = float(action[1].item())
                return linear, angular
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"Policy inference failed, using heuristic fallback: {exc}"
                )

        # Heuristic fallback: slow forward, turn away from closest obstacle
        if not scan.ranges:
            return 0.0, 0.0

        min_range = min(scan.ranges)
        min_index = scan.ranges.index(min_range)
        angle = scan.angle_min + min_index * scan.angle_increment

        if min_range < max(scan.range_min, 0.2):
            # Very close obstacle: stop
            return 0.0, 0.0
        if min_range < 0.8:
            # Turn away from obstacle
            direction = -1.0 if angle > 0 else 1.0
            return 0.0, direction * self.angular_speed

        # Occasionally add small angular noise to avoid getting stuck
        jitter = (random.random() - 0.5) * 0.1
        return self.linear_speed, jitter


class IntentPredictNode(Node):
    """
    Sensor fusion + intent prediction:
    - Subscribes to camera detections, depth motion, LiDAR clusters, IMU dynamics.
    - Publishes a discrete intent decision (slow_down/reroute/yield/proceed) and optional Twist.
    """

    def __init__(self):
        super().__init__("shobot_rl_intent")
        self.declare_parameter("model_path", "")
        self.declare_parameter("bbox_topic", "/detections")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("depth_topic", "/depth/image_raw")
        self.declare_parameter("decision_topic", "/rl_intent/decision")
        self.declare_parameter("cmd_vel_topic", "/rl_intent/cmd_vel")
        self.declare_parameter("base_linear_speed", 0.25)
        self.declare_parameter("base_angular_speed", 0.4)

        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        bbox_topic = self.get_parameter("bbox_topic").get_parameter_value().string_value
        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value
        depth_topic = (
            self.get_parameter("depth_topic").get_parameter_value().string_value
        )
        decision_topic = (
            self.get_parameter("decision_topic").get_parameter_value().string_value
        )
        cmd_vel_topic = (
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        )
        self.base_linear = (
            self.get_parameter("base_linear_speed").get_parameter_value().double_value
        )
        self.base_angular = (
            self.get_parameter("base_angular_speed").get_parameter_value().double_value
        )

        self.latest_bbox: Optional[Detection2DArray] = None
        self.latest_scan: Optional[LaserScan] = None
        self.latest_imu: Optional[Imu] = None
        self.latest_depth: Optional[Image] = None

        self.create_subscription(Detection2DArray, bbox_topic, self._bbox_cb, 5)
        self.create_subscription(LaserScan, scan_topic, self._scan_cb, 10)
        self.create_subscription(Imu, imu_topic, self._imu_cb, 10)
        self.create_subscription(Image, depth_topic, self._depth_cb, 5)
        self.decision_pub = self.create_publisher(String, decision_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_timer(0.2, self._tick)  # 5 Hz

        self.policy = self._load_policy(model_path)

        self.get_logger().info(
            f"Intent node started. Model: {model_path if model_path else 'heuristic'} "
            f"bbox: {bbox_topic}, scan: {scan_topic}, "
            f"imu: {imu_topic}, depth: {depth_topic}, decision: {decision_topic}"
        )

    def _bbox_cb(self, msg: Detection2DArray):
        self.latest_bbox = msg

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def _imu_cb(self, msg: Imu):
        self.latest_imu = msg

    def _depth_cb(self, msg: Image):
        self.latest_depth = msg

    def _tick(self):
        if self.latest_scan is None:
            return

        decision = self._infer_decision()
        cmd = Twist()

        if decision == "slow_down":
            cmd.linear.x = 0.5 * self.base_linear
        elif decision == "reroute":
            cmd.linear.x = 0.0
            cmd.angular.z = self.base_angular
        elif decision == "yield":
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = self.base_linear

        self.decision_pub.publish(String(data=decision))
        self.cmd_pub.publish(cmd)

    def _infer_decision(self) -> str:
        # Prepare features for policy: [min_range, human_density, wz, depth_flag]
        min_range = 5.0
        if self.latest_scan and self.latest_scan.ranges:
            min_range = min(self.latest_scan.ranges)

        human_density = len(self.latest_bbox.detections) if self.latest_bbox else 0
        wz = self.latest_imu.angular_velocity.z if self.latest_imu else 0.0
        depth_flag = 1.0 if self.latest_depth is not None else 0.0

        if self.policy is not None:
            try:
                import torch  # type: ignore

                feat = torch.tensor(
                    [[min_range, float(human_density), wz, depth_flag]],
                    dtype=torch.float32,
                )
                with torch.no_grad():
                    logits = self.policy(feat).squeeze(0)
                action_idx = int(torch.argmax(logits).item())
                return ["proceed", "slow_down", "yield", "reroute"][action_idx]
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"Intent policy inference failed, fallback heuristic: {exc}"
                )

        # Heuristic fallback
        # Use LiDAR to gauge proximity, IMU to detect sudden motion, bbox count as human density proxy.
        close_obstacle = False
        if self.latest_scan and self.latest_scan.ranges:
            min_range = min(self.latest_scan.ranges)
            close_obstacle = min_range < max(self.latest_scan.range_min, 0.7)

        human_density = len(self.latest_bbox.detections) if self.latest_bbox else 0
        high_human_density = human_density >= 3

        high_angular_rate = False
        if self.latest_imu:
            wz = self.latest_imu.angular_velocity.z
            high_angular_rate = abs(wz) > 0.7

        # Depth motion heuristic is approximated by presence of depth frames.
        depth_available = self.latest_depth is not None

        if close_obstacle and high_human_density:
            return "yield"
        if high_human_density or high_angular_rate:
            return "slow_down"
        if not depth_available and close_obstacle:
            return "reroute"
        return "proceed"

    def _load_policy(self, model_path: str):
        if not model_path:
            return None
        path = Path(model_path)
        if not path.exists():
            self.get_logger().warn(
                f"Intent model_path '{model_path}' not found; using heuristic."
            )
            return None
        try:
            import torch  # type: ignore

            policy = torch.jit.load(str(path))
            policy.eval()
            self.get_logger().info(f"Loaded intent TorchScript policy from {path}")
            return policy
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to load intent model '{path}': {exc}")
            return None


class SpeedModulationNode(Node):
    """
    Dynamic speed modulation:
    - Subscribes to obstacle density (LaserScan), curvature ahead (Path),
    floor conditions, and human proximity.
    - Modulates speed only; steering is untouched.
    - Publishes scaled Twist and a speed multiplier for visibility.
    """

    def __init__(self):
        super().__init__("shobot_rl_speed")
        self.declare_parameter("model_path", "")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("path_topic", "/plan")
        self.declare_parameter("floor_topic", "/floor_friction")
        self.declare_parameter("proximity_topic", "/humans/proximity")
        self.declare_parameter("input_cmd_topic", "/cmd_vel_raw")
        self.declare_parameter("output_cmd_topic", "/cmd_vel_modulated")
        self.declare_parameter("max_speed", 0.4)
        self.declare_parameter("min_speed", 0.05)

        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        path_topic = self.get_parameter("path_topic").get_parameter_value().string_value
        floor_topic = (
            self.get_parameter("floor_topic").get_parameter_value().string_value
        )
        proximity_topic = (
            self.get_parameter("proximity_topic").get_parameter_value().string_value
        )
        input_cmd_topic = (
            self.get_parameter("input_cmd_topic").get_parameter_value().string_value
        )
        output_cmd_topic = (
            self.get_parameter("output_cmd_topic").get_parameter_value().string_value
        )
        self.max_speed = (
            self.get_parameter("max_speed").get_parameter_value().double_value
        )
        self.min_speed = (
            self.get_parameter("min_speed").get_parameter_value().double_value
        )

        self.latest_scan: Optional[LaserScan] = None
        self.latest_path: Optional[Path] = None
        self.floor_coeff: Optional[float] = None
        self.human_proximity: Optional[float] = None
        self.latest_cmd: Optional[Twist] = None

        self.create_subscription(LaserScan, scan_topic, self._scan_cb, 10)
        self.create_subscription(Path, path_topic, self._path_cb, 5)
        self.create_subscription(Float32, floor_topic, self._floor_cb, 5)
        self.create_subscription(Float32, proximity_topic, self._proximity_cb, 5)
        self.create_subscription(Twist, input_cmd_topic, self._cmd_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, output_cmd_topic, 10)
        self.multiplier_pub = self.create_publisher(
            Float32, "/rl_speed/multiplier", 10
        )
        self.create_timer(0.05, self._tick)  # 20 Hz

        self.policy = self._load_policy(model_path)

        self.get_logger().info(
            f"Speed modulation node started. Model: {model_path if model_path else 'heuristic'} "
            f"in: {input_cmd_topic} out: {output_cmd_topic} "
            f"scan: {scan_topic}, path: {path_topic}, floor: {floor_topic}, proximity: {proximity_topic}"
        )

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def _path_cb(self, msg: Path):
        self.latest_path = msg

    def _floor_cb(self, msg: Float32):
        self.floor_coeff = msg.data

    def _proximity_cb(self, msg: Float32):
        self.human_proximity = msg.data

    def _cmd_cb(self, msg: Twist):
        self.latest_cmd = msg

    def _tick(self):
        if self.latest_cmd is None:
            return
        multiplier = self._compute_multiplier()
        out = Twist()
        out.linear.x = multiplier * self.latest_cmd.linear.x
        out.angular.z = self.latest_cmd.angular.z  # steering unchanged
        self.cmd_pub.publish(out)
        self.multiplier_pub.publish(Float32(data=multiplier))

    def _compute_multiplier(self) -> float:
        # Features for policy: [min_range, avg_curvature, floor_coeff, human_proximity, commanded_speed]
        min_range = 5.0
        if self.latest_scan and self.latest_scan.ranges:
            min_range = min(self.latest_scan.ranges)

        avg_curvature = 0.0
        if self.latest_path and len(self.latest_path.poses) >= 3:
            poses = self.latest_path.poses
            heading_changes = []
            for i in range(1, len(poses) - 1):
                dx1 = poses[i].pose.position.x - poses[i - 1].pose.position.x
                dy1 = poses[i].pose.position.y - poses[i - 1].pose.position.y
                dx2 = poses[i + 1].pose.position.x - poses[i].pose.position.x
                dy2 = poses[i + 1].pose.position.y - poses[i].pose.position.y
                angle1 = math.atan2(dy1, dx1)
                angle2 = math.atan2(dy2, dx2)
                heading_changes.append(abs(angle2 - angle1))
            if heading_changes:
                avg_curvature = sum(heading_changes) / len(heading_changes)

        floor_coeff = self.floor_coeff if self.floor_coeff is not None else 1.0
        human_prox = self.human_proximity if self.human_proximity is not None else 10.0
        commanded_speed = abs(self.latest_cmd.linear.x) if self.latest_cmd else 0.0

        if self.policy is not None:
            try:
                import torch  # type: ignore

                feat = torch.tensor(
                    [[min_range, avg_curvature, floor_coeff, human_prox, commanded_speed]],
                    dtype=torch.float32,
                )
                with torch.no_grad():
                    multiplier = float(self.policy(feat).squeeze().item())
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"Speed policy inference failed, fallback heuristic: {exc}"
                )
                multiplier = self._heuristic_multiplier(
                    min_range, avg_curvature, floor_coeff, human_prox
                )
        else:
            multiplier = self._heuristic_multiplier(
                min_range, avg_curvature, floor_coeff, human_prox
            )

        # Clamp to limits
        max_scale = 1.0
        min_scale = max(self.min_speed / max(abs(self.latest_cmd.linear.x), 1e-3), 0.0)
        multiplier = max(min(multiplier, max_scale), min_scale)
        # Also cap absolute speed
        desired_speed = abs(multiplier * self.latest_cmd.linear.x)
        if desired_speed > self.max_speed:
            multiplier = math.copysign(self.max_speed, multiplier) / max(
                abs(self.latest_cmd.linear.x), 1e-3
            )
        return float(multiplier)

    def _heuristic_multiplier(
        self, min_range: float, avg_curvature: float, floor_coeff: float, human_prox: float
    ) -> float:
        multiplier = 1.0
        if min_range < 0.5:
            multiplier *= 0.3
        elif min_range < 1.0:
            multiplier *= 0.6

        if avg_curvature > 0.6:
            multiplier *= 0.5
        elif avg_curvature > 0.3:
            multiplier *= 0.8

        if floor_coeff < 0.3:
            multiplier *= 0.5
        elif floor_coeff < 0.6:
            multiplier *= 0.8

        if human_prox < 0.8:
            multiplier *= 0.4
        elif human_prox < 1.5:
            multiplier *= 0.7
        return multiplier

    def _load_policy(self, model_path: str):
        if not model_path:
            return None
        path = Path(model_path)
        if not path.exists():
            self.get_logger().warn(
                f"Speed model_path '{model_path}' not found; using heuristic."
            )
            return None
        try:
            import torch  # type: ignore

            policy = torch.jit.load(str(path))
            policy.eval()
            self.get_logger().info(f"Loaded speed TorchScript policy from {path}")
            return policy
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to load speed model '{path}': {exc}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = RLAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


    if __name__ == "__main__":
        main()


def main_intent(args=None):
    rclpy.init(args=args)
    node = IntentPredictNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_speed(args=None):
    rclpy.init(args=args)
    node = SpeedModulationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
