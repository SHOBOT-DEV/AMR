import os
from pathlib import Path

# Base directory
basedir = Path(__file__).parent.absolute()


class Config:
    """Base configuration class"""

    SECRET_KEY = os.environ.get(
        "SECRET_KEY", "your-secret-key-here-change-in-production"
    )
    ACCESS_TOKEN_EXPIRES_HOURS = int(os.environ.get("ACCESS_TOKEN_EXPIRES_HOURS", 24))
    REFRESH_TOKEN_EXPIRES_DAYS = int(os.environ.get("REFRESH_TOKEN_EXPIRES_DAYS", 30))
    SQLALCHEMY_TRACK_MODIFICATIONS = False

    # Database configuration
    instance_path = os.path.join(basedir, "instance")
    os.makedirs(instance_path, exist_ok=True)
    SQLALCHEMY_DATABASE_URI = f"sqlite:///{os.path.join(instance_path, 'users.db')}"


class DevelopmentConfig(Config):
    """Development configuration"""

    DEBUG = True


class ProductionConfig(Config):
    """Production configuration"""

    DEBUG = False

    def __init__(self):
        super().__init__()
        if not self.SECRET_KEY:
            raise ValueError(
                "SECRET_KEY environment variable must be set in production"
            )


config = {
    "development": DevelopmentConfig,
    "production": ProductionConfig,
    "default": DevelopmentConfig,
}
