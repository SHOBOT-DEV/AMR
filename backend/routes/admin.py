from flask import Blueprint, jsonify, current_app
from backend import db
from backend.models import User
from backend.routes.auth import token_required

admin_bp = Blueprint("admin", __name__)


@admin_bp.route("/users", methods=["GET"])
@token_required
def get_users(current_user, user_role):
    if user_role != "admin":
        return jsonify({"message": "Admin access required!"}), 403

    users = User.query.all()
    users_list = [u.to_dict() for u in users]
    return jsonify(users_list)


@admin_bp.route("/users/<int:user_id>/approve", methods=["PUT"])
@token_required
def approve_user(current_user, user_role, user_id):
    if user_role != "admin":
        return jsonify({"message": "Admin access required!"}), 403

    try:
        user = User.query.get(user_id)
        if not user:
            return jsonify({"message": "User not found!", "success": False}), 404

        user.approval = "Approved"
        db.session.commit()

        return jsonify({"message": "User approved successfully!", "success": True})
    except Exception as e:
        current_app.logger.exception("Error in /api/admin/users/<id>/approve")
        return (
            jsonify({"message": "Server error", "detail": str(e), "success": False}),
            500,
        )


@admin_bp.route("/users/<int:user_id>/reject", methods=["PUT"])
@token_required
def reject_user(current_user, user_role, user_id):
    if user_role != "admin":
        return jsonify({"message": "Admin access required!"}), 403

    try:
        user = User.query.get(user_id)
        if not user:
            return jsonify({"message": "User not found!", "success": False}), 404

        user.approval = "Rejected"
        db.session.commit()

        return jsonify({"message": "User rejected successfully!", "success": True})
    except Exception as e:
        current_app.logger.exception("Error in /api/admin/users/<id>/reject")
        return (
            jsonify({"message": "Server error", "detail": str(e), "success": False}),
            500,
        )


@admin_bp.route("/users/<int:user_id>/pending", methods=["PUT"])
@token_required
def set_pending_user(current_user, user_role, user_id):
    if user_role != "admin":
        return jsonify({"message": "Admin access required!"}), 403

    try:
        user = User.query.get(user_id)
        if not user:
            return jsonify({"message": "User not found!", "success": False}), 404

        user.approval = "Pending"
        db.session.commit()

        return jsonify(
            {"message": "User status set to pending successfully!", "success": True}
        )
    except Exception as e:
        current_app.logger.exception("Error in /api/admin/users/<id>/pending")
        return (
            jsonify({"message": "Server error", "detail": str(e), "success": False}),
            500,
        )


@admin_bp.route("/users/<int:user_id>", methods=["DELETE"])
@token_required
def delete_user(current_user, user_role, user_id):
    if user_role != "admin":
        return jsonify({"message": "Admin access required!"}), 403

    try:
        user = User.query.get(user_id)
        if not user:
            return jsonify({"message": "User not found!", "success": False}), 404

        # Prevent admin from deleting themselves
        if user.username == current_user:
            return (
                jsonify(
                    {"message": "Cannot delete your own account!", "success": False}
                ),
                400,
            )

        db.session.delete(user)
        db.session.commit()

        return jsonify({"message": "User deleted successfully!", "success": True})
    except Exception as e:
        current_app.logger.exception("Error in /api/admin/users/<id> DELETE")
        return (
            jsonify({"message": "Server error", "detail": str(e), "success": False}),
            500,
        )
