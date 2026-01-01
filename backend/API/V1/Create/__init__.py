"""Register CRUD routes for the Create panel."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:  # pragma: no cover - hints only
    from flask import Blueprint
    from backend.API.V1.data_store import FrontendDataStore


def register_create_routes(bp: "Blueprint", store: "FrontendDataStore") -> None:
    from .Maps import register_maps_routes
    from .Zones import register_zone_routes
    from .WayPoints import register_waypoint_routes

    register_maps_routes(bp, store)
    register_zone_routes(bp, store)
    register_waypoint_routes(bp, store)


__all__ = ["register_create_routes"]
