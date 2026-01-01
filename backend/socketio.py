from __future__ import annotations

from flask_socketio import SocketIO

socketio = SocketIO(cors_allowed_origins="*")


def init_socketio(app) -> None:
    socketio.init_app(app)
    # Register event handlers after initialization to avoid circular imports.
    import backend.ws_handlers  # noqa: F401
