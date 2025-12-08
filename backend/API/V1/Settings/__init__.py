"""Settings panel routes."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:  # pragma: no cover
    from flask import Blueprint
    from backend.API.V1.data_store import FrontendDataStore


def register_settings_routes(bp: "Blueprint", store: "FrontendDataStore") -> None:
    from .robot import register_robot_settings_routes
    from .Account import register_account_routes
    from .Appearance import register_appearance_routes
    from .Security import register_security_routes
    from .Integrations import register_integration_routes

    register_robot_settings_routes(bp, store)
    register_account_routes(bp, store)
    register_appearance_routes(bp, store)
    register_security_routes(bp, store)
    register_integration_routes(bp, store)


__all__ = ["register_settings_routes"]
