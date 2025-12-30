from flask import Blueprint

from .data_store import FrontendDataStore
from .Create import register_create_routes
from .Monitor import register_monitor_routes
from .Settings import register_settings_routes
from .Chat.Chat import register_chat_routes
from .Stats.Stats import register_stats_routes
from .Legacy import register_legacy_routes


api_v1_bp = Blueprint("api_v1", __name__)
frontend_store = FrontendDataStore()


register_create_routes(api_v1_bp, frontend_store)
register_monitor_routes(api_v1_bp, frontend_store)
register_settings_routes(api_v1_bp, frontend_store)
register_chat_routes(api_v1_bp, frontend_store)
register_stats_routes(api_v1_bp, frontend_store)
register_legacy_routes(api_v1_bp, frontend_store)


__all__ = ["api_v1_bp", "frontend_store"]
