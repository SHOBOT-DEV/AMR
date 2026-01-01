from __future__ import annotations

from flask import jsonify, request
from werkzeug.security import check_password_hash

from backend import db
from backend.models import User, RefreshToken
from backend.routes.auth import token_required, _issue_token_pair


def register_auth_routes(bp, store):
    @bp.route("/auth/login", methods=["POST"])
    def auth_login():
        data = request.get_json(silent=True) or {}
        username = data.get("username")
        email = data.get("email")
        password = data.get("password")

        if not password or not (username or email):
            return jsonify({"success": False, "message": "Missing credentials"}), 400

        user = None
        if username:
            user = User.query.filter_by(username=username).first()
        if not user and email:
            user = User.query.filter_by(email=email).first()

        if not user or not check_password_hash(user.password_hash, password):
            return jsonify({"success": False, "message": "Invalid credentials"}), 401

        if user.role != "admin" and user.approval != "Approved":
            return (
                jsonify(
                    {
                        "success": False,
                        "message": "Your account is pending approval.",
                    }
                ),
                403,
            )

        tokens = _issue_token_pair(user)
        return jsonify(
            {
                "success": True,
                "token": tokens["token"],
                "refreshToken": tokens["refresh_token"],
                "tokenExpiresAt": tokens["token_expires_at"],
                "refreshTokenExpiresAt": tokens["refresh_expires_at"],
                "user": user.to_dict(),
            }
        )

    @bp.route("/auth/logout", methods=["POST"])
    @token_required
    def auth_logout(current_user, user_role):
        data = request.get_json(silent=True) or {}
        refresh_value = (
            data.get("refreshToken")
            or data.get("refresh_token")
            or request.headers.get("X-Refresh-Token")
        )

        user = User.query.filter_by(username=current_user).first()
        if not user:
            return jsonify({"success": False, "message": "User not found"}), 404

        if refresh_value:
            token = RefreshToken.query.filter_by(token=refresh_value).first()
            if token:
                token.revoked = True
                db.session.commit()
        else:
            RefreshToken.query.filter_by(user_id=user.id, revoked=False).update(
                {"revoked": True}, synchronize_session=False
            )
            db.session.commit()

        return jsonify({"success": True})

    @bp.route("/auth/me", methods=["GET"])
    @token_required
    def auth_me(current_user, user_role):
        user = User.query.filter_by(username=current_user).first()
        if not user:
            return jsonify({"success": False, "message": "User not found"}), 404
        return jsonify({"success": True, "user": user.to_dict()})


__all__ = ["register_auth_routes"]
