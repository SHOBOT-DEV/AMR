from flask import Blueprint

from .data_store import FrontendDataStore
from .Create import register_create_routes
from .Monitor import register_monitor_routes
from .Settings import register_settings_routes
from .Chat.Chat import register_chat_routes
from .Stats.Stats import register_stats_routes
from .ROS2 import register_ros2_routes
from .Video import register_video_routes
from .Integrations import register_integration_routes
from .Users import register_user_routes
from .Auth import register_auth_routes
from .LogsBags import register_log_bag_routes
from .System import register_system_routes
from .Parameters import register_parameter_routes
from .Costmap import register_costmap_routes
from .Sensors import register_sensor_routes
from .Perception import register_perception_routes
from .Recovery import register_recovery_routes
from .Safety import register_safety_routes
from .Docking import register_docking_routes
from .Missions import register_mission_routes


api_v1_bp = Blueprint("api_v1", __name__)
frontend_store = FrontendDataStore()


register_create_routes(api_v1_bp, frontend_store)
register_monitor_routes(api_v1_bp, frontend_store)
register_settings_routes(api_v1_bp, frontend_store)
register_chat_routes(api_v1_bp, frontend_store)
register_stats_routes(api_v1_bp, frontend_store)
register_ros2_routes(api_v1_bp, frontend_store)
register_video_routes(api_v1_bp, frontend_store)
register_integration_routes(api_v1_bp, frontend_store)
register_user_routes(api_v1_bp, frontend_store)
register_auth_routes(api_v1_bp, frontend_store)
register_log_bag_routes(api_v1_bp, frontend_store)
register_system_routes(api_v1_bp, frontend_store)
register_parameter_routes(api_v1_bp, frontend_store)
register_costmap_routes(api_v1_bp, frontend_store)
register_sensor_routes(api_v1_bp, frontend_store)
register_perception_routes(api_v1_bp, frontend_store)
register_recovery_routes(api_v1_bp, frontend_store)
register_safety_routes(api_v1_bp, frontend_store)
register_docking_routes(api_v1_bp, frontend_store)
register_mission_routes(api_v1_bp, frontend_store)


__all__ = ["api_v1_bp", "frontend_store"]
