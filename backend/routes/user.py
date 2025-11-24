from flask import Blueprint, request, jsonify, current_app
from backend import db
from backend.models import User
from backend.routes.auth import token_required
from werkzeug.security import generate_password_hash

user_bp = Blueprint("user", __name__)


@user_bp.route("/profile", methods=["GET"])
@token_required
def get_user_profile(current_user, user_role):
    try:
        user = User.query.filter_by(username=current_user).first()
        if not user:
            return jsonify({"message": "User not found!", "success": False}), 404

        return jsonify(
            {
                "username": user.username,
                "email": user.email,
                "role": user.role,
                "approval": user.approval,
                "company": user.company,
                "amr_type": user.amr_type,
                "success": True,
            }
        )
    except Exception as e:
        current_app.logger.exception("Error in /api/user/profile")
        return (
            jsonify({"message": "Server error", "detail": str(e), "success": False}),
            500,
        )


@user_bp.route("/profile", methods=["PUT"])
@token_required
def update_user_profile(current_user, user_role):
    try:
        data = request.get_json(silent=True)
        if not data:
            data = request.form.to_dict() if request.form else {}

        user = User.query.filter_by(username=current_user).first()
        if not user:
            return jsonify({"message": "User not found!", "success": False}), 404

        # Update allowed fields
        if "email" in data:
            # Check if email is already taken by another user
            existing_user = User.query.filter(
                User.email == data["email"], User.id != user.id
            ).first()
            if existing_user:
                return (
                    jsonify({"message": "Email already in use!", "success": False}),
                    400,
                )
            user.email = data["email"]

        if "company" in data:
            user.company = data["company"]

        if "amr_type" in data:
            user.amr_type = (
                data["amrType"] if "amrType" in data else data.get("amr_type")
            )

        db.session.commit()

        return jsonify({"message": "Profile updated successfully!", "success": True})
    except Exception as e:
        current_app.logger.exception("Error in /api/user/profile PUT")
        return (
            jsonify({"message": "Server error", "detail": str(e), "success": False}),
            500,
        )


@user_bp.route("/password", methods=["PUT"])
@token_required
def update_password(current_user, user_role):
    try:
        data = request.get_json(silent=True)
        if not data:
            data = request.form.to_dict() if request.form else {}

        old_password = data.get("oldPassword")
        new_password = data.get("newPassword")

        if not old_password or not new_password:
            return (
                jsonify(
                    {
                        "message": "Old password and new password required!",
                        "success": False,
                    }
                ),
                400,
            )

        user = User.query.filter_by(username=current_user).first()
        if not user:
            return jsonify({"message": "User not found!", "success": False}), 404

        if not user.check_password(old_password):
            return (
                jsonify({"message": "Incorrect old password!", "success": False}),
                401,
            )

        user.password_hash = generate_password_hash(new_password)
        db.session.commit()

        return jsonify({"message": "Password updated successfully!", "success": True})
    except Exception as e:
        current_app.logger.exception("Error in /api/user/password")
        return (
            jsonify({"message": "Server error", "detail": str(e), "success": False}),
            500,
        )
