from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:  # pragma: no cover
    from flask import Blueprint
    from backend.API.V1.data_store import FrontendDataStore


def register_video_routes(bp: "Blueprint", store: "FrontendDataStore") -> None:
    from .Streams import register_video_routes as _register

    _register(bp, store)


__all__ = ["register_video_routes"]
