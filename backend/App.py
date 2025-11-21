from flask import Flask, request, jsonify
from flask_sqlalchemy import SQLAlchemy
import jwt
import datetime
from functools import wraps
from werkzeug.security import generate_password_hash, check_password_hash


app = Flask(__name__)

app.config['SECRET_KEY'] = 'your-secret-key-here'  # Change this to a strong secret
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///users.db'  # SQLite DB for simplicity
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False

db = SQLAlchemy(app)

class User(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    username = db.Column(db.String(50), unique=True, nullable=False)
    email = db.Column(db.String(120), unique=True, nullable=False)
    password_hash = db.Column(db.String(200), nullable=False)
    role = db.Column(db.String(20), default='user')
    approval = db.Column(db.String(20), default='Pending')

    def check_password(self, password):
        return check_password_hash(self.password_hash, password)

# Create the database tables
with app.app_context():
    db.create_all()


def token_required(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        token = request.headers.get("Authorization")
        if not token:
            return jsonify({"message": "Token is missing!"}), 401

        if token.startswith("Bearer "):
            token = token.split(" ")[1]

        try:
            decoded = jwt.decode(token, app.config["SECRET_KEY"], algorithms=["HS256"])
            current_user = decoded.get("user")
            user_role = decoded.get("role")
        except jwt.ExpiredSignatureError:
            return jsonify({"message": "Token has expired!"}), 401
        except jwt.InvalidTokenError:
            return jsonify({"message": "Token is invalid!"}), 401

        return func(current_user, user_role, *args, **kwargs)
    return wrapper


@app.route("/api/register", methods=["POST"])
def register():
    try:
        # try parse JSON first, allow silent so it returns None if not JSON
        data = request.get_json(silent=True)
        if not data:
            # fallback to form-encoded data (e.g. application/x-www-form-urlencoded)
            data = request.form.to_dict() if request.form else {}

        # Debugging info (check your server logs when POST fails)
        app.logger.debug("Register request headers: %s", dict(request.headers))
        app.logger.debug("Register request data: %s", data)

        if not data:
            return jsonify({"message": "No JSON body received. Ensure Content-Type: application/json", "success": False}), 400

        username = data.get("username")
        email = data.get("email")
        password = data.get("password")

        if not username or not email or not password:
            return jsonify({"message": "Missing username, email or password", "success": False}), 400

        if User.query.filter((User.username == username) | (User.email == email)).first():
            return jsonify({"message": "Username or email already exists!", "success": False}), 400

        user = User(
            username=username,
            email=email,
            password_hash=generate_password_hash(password),
            role="user",
            approval="Pending"
        )
        db.session.add(user)
        db.session.commit()

        return jsonify({"message": "Registration successful!", "success": True})
    except Exception as e:
        app.logger.exception("Error in /api/register")
        return jsonify({"message": "Server error", "detail": str(e), "success": False}), 500


@app.route("/api/login", methods=["POST"])
def login():
    try:
        data = request.get_json(silent=True)
        if not data:
            data = request.form.to_dict() if request.form else {}

        # Debugging info
        app.logger.debug("Login request headers: %s", dict(request.headers))
        app.logger.debug("Login request data: %s", data)

        if not data:
            return jsonify({"message": "No JSON body received. Ensure Content-Type: application/json", "success": False}), 400

        username = data.get("username")
        password = data.get("password")

        if not username or not password:
            return jsonify({"message": "Missing username or password", "success": False}), 400

        user = User.query.filter_by(username=username).first()
        if user and user.check_password(password):
            token = jwt.encode({
                "user": user.username,
                "role": user.role,
                "exp": datetime.datetime.utcnow() + datetime.timedelta(hours=24)
            }, app.config["SECRET_KEY"], algorithm="HS256")

            # ensure token is a string (pyjwt may return bytes on some versions)
            if isinstance(token, bytes):
                token = token.decode("utf-8")

            return jsonify({
                "token": token,
                "user": {"username": user.username, "role": user.role},
                "success": True
            })

        return jsonify({"message": "Invalid credentials!", "success": False}), 401
    except Exception as e:
        app.logger.exception("Error in /api/login")
        return jsonify({"message": "Server error", "detail": str(e), "success": False}), 500


@app.route("/api/protected", methods=["GET"])
@token_required
def protected(current_user, user_role):
    return jsonify({"message": f"Hello {current_user}, this is a protected route!"})

@app.route("/api/public", methods=["GET"])
def public():
    return jsonify({"message": "This is a public route!"})


@app.route("/api/admin/users", methods=["GET"])
@token_required
def get_users(current_user, user_role):
    if user_role != "admin":
        return jsonify({"message": "Admin access required!"}), 403

    users = User.query.all()
    users_list = [
        {"username": u.username, "email": u.email, "role": u.role, "approval": u.approval}
        for u in users
    ]
    return jsonify(users_list)

@app.route('/')
def home():
    return "Hello, Flask is running!"


if __name__ == "__main__":
    app.run(debug=True, port=5000)

