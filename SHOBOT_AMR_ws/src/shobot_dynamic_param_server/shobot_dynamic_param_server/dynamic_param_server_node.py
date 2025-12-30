#!/usr/bin/env python3
"""
Dynamic Parameter Server Node.

Exposes runtime-tunable parameters (speed limits, safety thresholds,
costmap inflation, laser filter ranges, Nav2 replanning thresholds).

Parameters can be changed live via `ros2 param set` without restarting nodes.
Optionally propagates validated changes to other nodes using SetParameters.
"""

import threading
from dataclasses import dataclass
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rcl_interfaces.msg import (
    SetParametersResult,
    ParameterDescriptor,
    FloatingPointRange,
)


# ----------------------------------------------------------------------
# Parameter specification
# ----------------------------------------------------------------------
@dataclass(frozen=True)
class ParamSpec:
    name: str
    default: float
    min_value: float
    max_value: float
    description: str


# ----------------------------------------------------------------------
class DynamicParamServerNode(Node):
    """Runtime parameter server with validation and optional propagation."""

    def __init__(self):
        super().__init__("shobot_dynamic_param_server")

        # ---------------- Meta-parameters ----------------
        self.declare_parameter("propagate_targets", [])
        self.propagate_targets = self._normalize_list(
            self.get_parameter("propagate_targets").value
        )

        # ---------------- Parameter specs ----------------
        self.param_specs: Dict[str, ParamSpec] = self._build_param_specs()

        # ---------------- Internal state ----------------
        self._clients: Dict[str, AsyncParameterClient] = {}
        self._lock = threading.Lock()

        # ---------------- Declare parameters ----------------
        for spec in self.param_specs.values():
            descriptor = ParameterDescriptor(
                description=spec.description,
                floating_point_range=[
                    FloatingPointRange(
                        from_value=spec.min_value,
                        to_value=spec.max_value,
                        step=0.0,
                    )
                ],
            )
            self.declare_parameter(spec.name, spec.default, descriptor)

        # ---------------- Callbacks ----------------
        self.add_on_set_parameters_callback(self._on_param_set)

        self.get_logger().info(
            "Dynamic parameter server ready. "
            f"Propagation targets: {self.propagate_targets}"
        )

    # ------------------------------------------------------------------
    def _build_param_specs(self) -> Dict[str, ParamSpec]:
        specs = [
            ParamSpec("speed_limit_linear", 0.6, 0.0, 3.0, "Max linear speed (m/s)."),
            ParamSpec("speed_limit_angular", 1.5, 0.0, 6.0, "Max angular speed (rad/s)."),
            ParamSpec("safety_stop_distance", 0.5, 0.0, 5.0, "Emergency stop distance (m)."),
            ParamSpec("safety_slowdown_distance", 1.0, 0.0, 10.0, "Slowdown distance (m)."),
            ParamSpec("costmap_inflation_radius", 0.7, 0.0, 5.0, "Costmap inflation radius (m)."),
            ParamSpec("costmap_inscribed_radius", 0.3, 0.0, 3.0, "Robot inscribed radius (m)."),
            ParamSpec("laser_min_range", 0.05, 0.0, 2.0, "Laser minimum range (m)."),
            ParamSpec("laser_max_range", 12.0, 0.5, 100.0, "Laser maximum range (m)."),
            ParamSpec("nav2_replan_dist_threshold", 0.25, 0.0, 5.0, "Nav2 replan distance (m)."),
            ParamSpec("nav2_replan_time_threshold", 1.0, 0.0, 30.0, "Nav2 replan time (s)."),
        ]
        return {spec.name: spec for spec in specs}

    # ------------------------------------------------------------------
    def _normalize_list(self, val) -> List[str]:
        if val is None:
            return []
        if isinstance(val, list):
            return [str(v).strip() for v in val if str(v).strip()]
        if isinstance(val, str):
            text = val.strip()
            if text.startswith("[") and text.endswith("]"):
                inner = text[1:-1]
                return [
                    item.strip().strip("\"'")
                    for item in inner.split(",")
                    if item.strip().strip("\"'")
                ]
            if text:
                return [text]
        return []

    # ------------------------------------------------------------------
    def _on_param_set(self, params: List[Parameter]) -> SetParametersResult:
        """Validate parameter updates before they are applied."""
        validated: Dict[str, float] = {}

        for param in params:
            spec = self.param_specs.get(param.name)
            if spec is None:
                # Allow unrelated parameters
                continue

            if param.type_ != Parameter.Type.DOUBLE:
                return SetParametersResult(
                    successful=False,
                    reason=f"Parameter '{param.name}' must be a floating-point value.",
                )

            if not (spec.min_value <= param.value <= spec.max_value):
                return SetParametersResult(
                    successful=False,
                    reason=(
                        f"Parameter '{param.name}' out of range "
                        f"[{spec.min_value}, {spec.max_value}]"
                    ),
                )

            validated[param.name] = float(param.value)

        # Schedule propagation asynchronously (never block callback)
        if validated and self.propagate_targets:
            threading.Thread(
                target=self._propagate, args=(validated,), daemon=True
            ).start()

        for name, value in validated.items():
            self.get_logger().info(f"Parameter updated: {name} = {value}")

        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    def _propagate(self, changes: Dict[str, float]):
        """Propagate parameter changes to target nodes."""
        with self._lock:
            for node_name in self.propagate_targets:
                client = self._clients.get(node_name)
                if client is None:
                    client = AsyncParameterClient(self, node_name=node_name)
                    self._clients[node_name] = client

                params = [Parameter(name=k, value=v) for k, v in changes.items()]

                try:
                    future = client.set_parameters(params)
                    rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                    result = future.result()

                    if result is None or not all(r.successful for r in result):
                        self.get_logger().warn(
                            f"Parameter propagation to '{node_name}' failed: {result}"
                        )
                except Exception as exc:
                    self.get_logger().warn(
                        f"Exception while propagating to '{node_name}': {exc}"
                    )


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = DynamicParamServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
