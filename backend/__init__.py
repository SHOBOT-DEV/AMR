from flask import Flask
from flask_sqlalchemy import SQLAlchemy
from flask_cors import CORS
from backend.config import config

# Initialize extensions
db = SQLAlchemy()
cors = CORS()


def create_app(config_name="default"):
    """Application factory pattern"""
    app = Flask(__name__)

    # Load configuration
    app.config.from_object(config[config_name])

    # Initialize extensions
    db.init_app(app)
    cors.init_app(app)

    # Register blueprints
    from backend.routes.auth import auth_bp
    from backend.routes.admin import admin_bp
    from backend.routes.user import user_bp
    from backend.routes.stats import stats_bp
    from backend.API.V1 import api_v1_bp

    app.register_blueprint(auth_bp, url_prefix="/api")
    app.register_blueprint(admin_bp, url_prefix="/api/admin")
    app.register_blueprint(user_bp, url_prefix="/api/user")
    app.register_blueprint(stats_bp, url_prefix="/api")
    app.register_blueprint(api_v1_bp, url_prefix="/api/v1")

    # Register root route
    @app.route("/")
    def home():
        return "Hello, Flask is running!"

    # Create database tables
    with app.app_context():
        db.create_all()
        # Create default admin user if it doesn't exist
        from backend.models import User
        from werkzeug.security import generate_password_hash

        admin_user = User.query.filter_by(username="admin").first()
        if not admin_user:
            admin_user = User(
                username="admin",
                email="admin@admin.com",
                password_hash=generate_password_hash("admin123"),
                role="admin",
                approval="Approved",
            )
            db.session.add(admin_user)
            db.session.commit()
            print("Default admin user created: username='admin', password='admin123'")

    return app
