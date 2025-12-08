"""Monitor panel routes."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:  # pragma: no cover
    from flask import Blueprint
    from backend.API.V1.data_store import FrontendDataStore


def register_monitor_routes(bp: "Blueprint", store: "FrontendDataStore") -> None:
    from .Analytics import register_analytics_routes
    from .Diagnostics import register_diagnostics_routes
    from .Logs import register_log_routes
    from .MissionLogs import register_mission_log_routes
    from .RobotBags import register_robot_bag_routes

    register_analytics_routes(bp, store)
    register_diagnostics_routes(bp, store)
    register_log_routes(bp, store)
    register_mission_log_routes(bp, store)
    register_robot_bag_routes(bp, store)


__all__ = ["register_monitor_routes"]
