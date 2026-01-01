"""
Main entry point for the Flask application
"""

import sys
import os

# Add parent directory to path to allow imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from backend import create_app
from backend.socketio_app import socketio

app = create_app()


def _str_to_bool(value: str) -> bool:
    return value.lower() not in {"0", "false", "no", "off"}


if __name__ == "__main__":
    port = int(os.environ.get("FLASK_PORT", os.environ.get("PORT", 5000)))
    host = os.environ.get("FLASK_HOST", "127.0.0.1")
    debug_env = os.environ.get("FLASK_DEBUG", os.environ.get("DEBUG", "1"))
    debug = _str_to_bool(str(debug_env)) if isinstance(debug_env, str) else bool(debug_env)

    socketio.run(
        app,
        debug=debug,
        host=host,
        port=port,
        allow_unsafe_werkzeug=True,
    )
