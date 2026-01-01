from __future__ import annotations

from flask import jsonify, request
from werkzeug.security import generate_password_hash

from backend import db
from backend.models import User
from backend.routes.auth import token_required


def _require_admin(user_role: str):
    if user_role != "admin":
        return jsonify({"success": False, "message": "Admin access required"}), 403
    return None


def register_user_routes(bp, store):
    @bp.route("/users", methods=["GET"])
    @token_required
    def list_users(current_user, user_role):
        denied = _require_admin(user_role)
        if denied:
            return denied
        users = [u.to_dict() for u in User.query.all()]
        return jsonify({"success": True, "items": users})

    @bp.route("/users", methods=["POST"])
    @token_required
    def create_user(current_user, user_role):
        denied = _require_admin(user_role)
        if denied:
            return denied
        data = request.get_json(silent=True) or {}
        username = data.get("username")
        email = data.get("email")
        password = data.get("password")
        if not username or not email or not password:
            return (
                jsonify(
                    {
                        "success": False,
                        "message": "username, email, and password are required",
                    }
                ),
                400,
            )

        if User.query.filter((User.username == username) | (User.email == email)).first():
            return jsonify({"success": False, "message": "User already exists"}), 400

        user = User(
            username=username,
            email=email,
            password_hash=generate_password_hash(password),
            role=data.get("role", "user"),
            approval=data.get("approval", "Pending"),
            company=data.get("company"),
            amr_type=data.get("amr_type") or data.get("amrType"),
        )
        db.session.add(user)
        db.session.commit()
        return jsonify({"success": True, "user": user.to_dict()}), 201

    @bp.route("/users/<int:user_id>", methods=["GET"])
    @token_required
    def get_user(current_user, user_role, user_id: int):
        user = User.query.get(user_id)
        if not user:
            return jsonify({"success": False, "message": "User not found"}), 404
        if user_role != "admin" and user.username != current_user:
            return jsonify({"success": False, "message": "Access denied"}), 403
        return jsonify({"success": True, "user": user.to_dict()})

    @bp.route("/users/<int:user_id>", methods=["PUT"])
    @token_required
    def update_user(current_user, user_role, user_id: int):
        denied = _require_admin(user_role)
        if denied:
            return denied
        user = User.query.get(user_id)
        if not user:
            return jsonify({"success": False, "message": "User not found"}), 404

        data = request.get_json(silent=True) or {}
        if "username" in data:
            user.username = data["username"]
        if "email" in data:
            user.email = data["email"]
        if "role" in data:
            user.role = data["role"]
        if "approval" in data:
            user.approval = data["approval"]
        if "company" in data:
            user.company = data["company"]
        if "amr_type" in data or "amrType" in data:
            user.amr_type = data.get("amr_type") or data.get("amrType")
        if "password" in data:
            user.password_hash = generate_password_hash(data["password"])

        db.session.commit()
        return jsonify({"success": True, "user": user.to_dict()})

    @bp.route("/users/<int:user_id>", methods=["DELETE"])
    @token_required
    def delete_user(current_user, user_role, user_id: int):
        denied = _require_admin(user_role)
        if denied:
            return denied
        user = User.query.get(user_id)
        if not user:
            return jsonify({"success": False, "message": "User not found"}), 404
        if user.username == current_user:
            return (
                jsonify(
                    {"success": False, "message": "Cannot delete your own account"}
                ),
                400,
            )
        db.session.delete(user)
        db.session.commit()
        return jsonify({"success": True})


__all__ = ["register_user_routes"]
