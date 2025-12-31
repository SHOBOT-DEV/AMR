from __future__ import annotations

from flask import jsonify, request


def register_ros2_robot_routes(bp, store):
    @bp.route("/robot/state", methods=["GET"])
    def robot_state():
        return jsonify({"state": store.get_robot_state()})

    @bp.route("/robot/pose", methods=["GET"])
    def robot_pose():
        snapshot = store.get_ros2_snapshot()
        return jsonify(snapshot.get("pose", {}))

    @bp.route("/robot/velocity", methods=["GET"])
    def robot_velocity():
        snapshot = store.get_ros2_snapshot()
        return jsonify(snapshot.get("velocity", {}))

    @bp.route("/robot/battery", methods=["GET"])
    def robot_battery():
        status = store.get_robot_battery_status()
        return jsonify(status)

    @bp.route("/robot/mode", methods=["GET"])
    def robot_mode():
        return jsonify({"mode": store.get_robot_mode()})

    @bp.route("/robot/stop", methods=["POST"])
    def robot_stop():
        store.set_ros2_safety_stop(True)
        store.set_robot_state("STOPPED")
        return jsonify({"success": True, "stop": True})

    @bp.route("/robot/reset", methods=["POST"])
    def robot_reset():
        store.set_robot_state("RESET")
        return jsonify({"success": True, "state": store.get_robot_state()})

    @bp.route("/robot/shutdown", methods=["POST"])
    def robot_shutdown():
        store.set_robot_state("SHUTDOWN")
        return jsonify({"success": True, "state": store.get_robot_state()})

    @bp.route("/robot/reboot", methods=["POST"])
    def robot_reboot():
        store.set_robot_state("REBOOT")
        return jsonify({"success": True, "state": store.get_robot_state()})

    @bp.route("/ros2/status", methods=["GET"])
    def ros2_status():
        snapshot = store.get_ros2_snapshot()
        return jsonify({"success": True, "data": snapshot})

    @bp.route("/ros2/robot/pose", methods=["GET"])
    def ros2_pose():
        snapshot = store.get_ros2_snapshot()
        return jsonify({"success": True, "pose": snapshot.get("pose", {})})

    @bp.route("/ros2/navigation/go_to_pose", methods=["POST"])
    def ros2_go_to_pose():
        payload = request.get_json(silent=True) or {}
        try:
            goal = {
                "x": float(payload.get("x", 0.0)),
                "y": float(payload.get("y", 0.0)),
                "theta": float(payload.get("theta", 0.0)),
            }
        except (TypeError, ValueError):
            return jsonify({"success": False, "message": "Invalid pose payload"}), 400

        stored = store.set_ros2_goal(goal)
        return jsonify({"success": True, "goal": stored})

    @bp.route("/ros2/control/cmd_vel", methods=["POST"])
    def ros2_cmd_vel():
        payload = request.get_json(silent=True) or {}
        try:
            cmd = {
                "linear_x": float(payload.get("linear_x", 0.0)),
                "angular_z": float(payload.get("angular_z", 0.0)),
            }
        except (TypeError, ValueError):
            return jsonify({"success": False, "message": "Invalid cmd_vel payload"}), 400

        stored = store.set_ros2_cmd_vel(cmd)
        return jsonify({"success": True, "command": stored})


__all__ = ["register_ros2_robot_routes"]
