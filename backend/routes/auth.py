from flask import Blueprint, request, jsonify, current_app
from backend import db
from backend.models import User, RefreshToken
from werkzeug.security import generate_password_hash
import jwt
import datetime
import secrets
from functools import wraps

auth_bp = Blueprint("auth", __name__)


def token_required(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        token = request.headers.get("Authorization")
        if not token:
            return jsonify({"message": "Token is missing!"}), 401

        if token.startswith("Bearer "):
            token = token.split(" ")[1]

        try:
            decoded = jwt.decode(
                token, current_app.config["SECRET_KEY"], algorithms=["HS256"]
            )
            current_user = decoded.get("user")
            user_role = decoded.get("role")
        except jwt.ExpiredSignatureError:
            return jsonify({"message": "Token has expired!"}), 401
        except jwt.InvalidTokenError:
            return jsonify({"message": "Token is invalid!"}), 401

        return func(current_user, user_role, *args, **kwargs)

    return wrapper


def _generate_access_token(user):
    """Create a short-lived access token for authenticated requests."""
    lifetime_hours = current_app.config.get("ACCESS_TOKEN_EXPIRES_HOURS", 24)
    expires_at = datetime.datetime.utcnow() + datetime.timedelta(hours=lifetime_hours)
    token = jwt.encode(
        {
            "user": user.username,
            "role": user.role,
            "exp": expires_at,
        },
        current_app.config["SECRET_KEY"],
        algorithm="HS256",
    )

    if isinstance(token, bytes):
        token = token.decode("utf-8")

    return token, expires_at


def _generate_refresh_token(user):
    """Rotate refresh tokens to support long-term logins."""
    lifetime_days = current_app.config.get("REFRESH_TOKEN_EXPIRES_DAYS", 30)
    expires_at = datetime.datetime.utcnow() + datetime.timedelta(days=lifetime_days)

    # Revoke any existing tokens for this user before minting a new one.
    RefreshToken.query.filter_by(user_id=user.id, revoked=False).update(
        {"revoked": True}, synchronize_session=False
    )

    token_value = secrets.token_urlsafe(64)
    refresh_token = RefreshToken(
        token=token_value,
        user_id=user.id,
        expires_at=expires_at,
    )
    db.session.add(refresh_token)
    db.session.commit()

    return token_value, expires_at


def _issue_token_pair(user):
    access_token, access_expires = _generate_access_token(user)
    refresh_token, refresh_expires = _generate_refresh_token(user)
    return {
        "token": access_token,
        "token_expires_at": access_expires.isoformat() + "Z",
        "refresh_token": refresh_token,
        "refresh_expires_at": refresh_expires.isoformat() + "Z",
    }


@auth_bp.route("/register", methods=["POST"])
def register():
    try:
        # Try parse JSON first, allow silent so it returns None if not JSON
        data = request.get_json(silent=True)
        if not data:
            # Fallback to form-encoded data (e.g. application/x-www-form-urlencoded)
            data = request.form.to_dict() if request.form else {}

        # Debugging info (check your server logs when POST fails)
        current_app.logger.debug("Register request headers: %s", dict(request.headers))
        current_app.logger.debug("Register request data: %s", data)

        if not data:
            return (
                jsonify(
                    {
                        "message": "No JSON body received. Ensure Content-Type: application/json",
                        "success": False,
                    }
                ),
                400,
            )

        username = data.get("username")
        email = data.get("email")
        password = data.get("password")
        company = data.get("company")
        amr_type = data.get("amrType")

        if not username or not email or not password:
            return (
                jsonify(
                    {"message": "Missing username, email or password", "success": False}
                ),
                400,
            )

        if User.query.filter(
            (User.username == username) | (User.email == email)
        ).first():
            return (
                jsonify(
                    {"message": "Username or email already exists!", "success": False}
                ),
                400,
            )

        user = User(
            username=username,
            email=email,
            password_hash=generate_password_hash(password),
            role="user",
            approval="Pending",
            company=company,
            amr_type=amr_type,
        )
        db.session.add(user)
        db.session.commit()

        return jsonify({"message": "Registration successful!", "success": True})
    except Exception as e:
        current_app.logger.exception("Error in /api/register")
        return (
            jsonify({"message": "Server error", "detail": str(e), "success": False}),
            500,
        )


@auth_bp.route("/login", methods=["POST"])
def login():
    try:
        data = request.get_json(silent=True)
        if not data:
            data = request.form.to_dict() if request.form else {}

        # Debugging info
        current_app.logger.debug("Login request headers: %s", dict(request.headers))
        current_app.logger.debug("Login request data: %s", data)

        if not data:
            return (
                jsonify(
                    {
                        "message": "No JSON body received. Ensure Content-Type: application/json",
                        "success": False,
                    }
                ),
                400,
            )

        username = data.get("username")
        password = data.get("password")

        if not username or not password:
            return (
                jsonify({"message": "Missing username or password", "success": False}),
                400,
            )

        user = User.query.filter_by(username=username).first()
        if user and user.check_password(password):
            # Check if user is approved (admin can always login)
            if user.role != "admin" and user.approval != "Approved":
                return (
                    jsonify(
                        {
                            "message": "Your account is pending approval. Please wait for admin approval.",
                            "success": False,
                        }
                    ),
                    403,
                )

            tokens = _issue_token_pair(user)

            return jsonify(
                {
                    "token": tokens["token"],
                    "refreshToken": tokens["refresh_token"],
                    "tokenExpiresAt": tokens["token_expires_at"],
                    "refreshTokenExpiresAt": tokens["refresh_expires_at"],
                    "user": {
                        "username": user.username,
                        "role": user.role,
                        "approval": user.approval,
                    },
                    "success": True,
                }
            )

        return jsonify({"message": "Invalid credentials!", "success": False}), 401
    except Exception as e:
        current_app.logger.exception("Error in /api/login")
        return (
            jsonify({"message": "Server error", "detail": str(e), "success": False}),
            500,
        )


@auth_bp.route("/refresh", methods=["POST"])
def refresh_login():
    """Exchange a valid refresh token for a new access token pair."""
    data = request.get_json(silent=True) or {}
    refresh_token_value = (
        data.get("refreshToken")
        or data.get("refresh_token")
        or request.headers.get("X-Refresh-Token")
    )

    if not refresh_token_value:
        return (
            jsonify({"message": "Refresh token is required", "success": False}),
            400,
        )

    token_record = RefreshToken.query.filter_by(
        token=refresh_token_value, revoked=False
    ).first()

    if not token_record or token_record.is_expired():
        if token_record and not token_record.revoked:
            token_record.revoked = True
            db.session.commit()
        return (
            jsonify({"message": "Invalid or expired refresh token", "success": False}),
            401,
        )

    user = token_record.user
    token_record.revoked = True

    tokens = _issue_token_pair(user)

    return jsonify(
        {
            "token": tokens["token"],
            "refreshToken": tokens["refresh_token"],
            "tokenExpiresAt": tokens["token_expires_at"],
            "refreshTokenExpiresAt": tokens["refresh_expires_at"],
            "success": True,
        }
    )


@auth_bp.route("/protected", methods=["GET"])
@token_required
def protected(current_user, user_role):
    return jsonify({"message": f"Hello {current_user}, this is a protected route!"})


@auth_bp.route("/public", methods=["GET"])
def public():
    return jsonify({"message": "This is a public route!"})


@auth_bp.route("/verify-token", methods=["GET"])
@token_required
def verify_token(current_user, user_role):
    """Verify if the token is valid and return user info"""
    try:
        user = User.query.filter_by(username=current_user).first()
        if not user:
            return jsonify({"message": "User not found!", "success": False}), 404

        return jsonify(
            {
                "valid": True,
                "user": {
                    "username": user.username,
                    "email": user.email,
                    "role": user.role,
                    "approval": user.approval,
                },
                "success": True,
            }
        )
    except Exception as e:
        current_app.logger.exception("Error in /api/verify-token")
        return (
            jsonify({"message": "Server error", "detail": str(e), "success": False}),
            500,
        )