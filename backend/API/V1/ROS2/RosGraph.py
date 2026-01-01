from __future__ import annotations

import subprocess
from shutil import which
from typing import List, Optional

from flask import jsonify


def _ros2_cli_available() -> bool:
    return which("ros2") is not None


def _run_ros2_list(args: List[str], timeout: float = 2.0) -> Optional[List[str]]:
    if not _ros2_cli_available():
        return None
    try:
        result = subprocess.run(
            ["ros2", *args],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if result.returncode != 0:
        return None
    return [line.strip() for line in result.stdout.splitlines() if line.strip()]


def _format_ros2_param_list(lines: Optional[List[str]]) -> Optional[List[str]]:
    if lines is None:
        return None
    params: List[str] = []
    current_node = ""
    for line in lines:
        if line.endswith(":") and not line.startswith(" "):
            current_node = line[:-1].strip()
            continue
        if current_node and line.startswith(" "):
            name = line.strip()
            if name:
                params.append(f"{current_node}.{name}")
            continue
        params.append(line.strip())
    return params


def register_ros_graph_routes(bp, store):
    @bp.route("/ros/nodes", methods=["GET"])
    def ros_nodes():
        nodes = _run_ros2_list(["node", "list"])
        return jsonify({"nodes": nodes if nodes is not None else store.list_ros_nodes()})

    @bp.route("/ros/topics", methods=["GET"])
    def ros_topics():
        topics = _run_ros2_list(["topic", "list"])
        return jsonify({"topics": topics if topics is not None else store.list_ros_topics()})

    @bp.route("/ros/services", methods=["GET"])
    def ros_services():
        services = _run_ros2_list(["service", "list"])
        return jsonify({"services": services if services is not None else store.list_ros_services()})

    @bp.route("/ros/actions", methods=["GET"])
    def ros_actions():
        actions = _run_ros2_list(["action", "list"])
        return jsonify({"actions": actions if actions is not None else store.list_ros_actions()})

    @bp.route("/ros/parameters", methods=["GET"])
    def ros_parameters():
        params = _format_ros2_param_list(_run_ros2_list(["param", "list"]))
        if params is None:
            params = sorted(store.list_ros2_parameters().keys())
        return jsonify({"parameters": params})


__all__ = ["register_ros_graph_routes"]
