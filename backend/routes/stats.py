from flask import Blueprint, jsonify

stats_bp = Blueprint("stats", __name__)


@stats_bp.route("/stats", methods=["GET"])
def get_dashboard_stats():
    """
    Returns aggregated dashboard metrics used by the frontend stats pane.
    In a production deployment these values should come from mission/telemetry
    tables or live robot streams. For now we provide representative sample data.
    """

    data = {
        "overview": {
            "totalKm": 182.4,
            "missionsCompleted": 47,
            "avgSpeed": 1.8,
            "operatingHours": 326,
            "deltaKm": 4.3,
            "missionSuccessRate": 98,
        },
        "missionTrend": [
            {"label": "Mon", "completed": 5, "incidents": 0},
            {"label": "Tue", "completed": 7, "incidents": 1},
            {"label": "Wed", "completed": 6, "incidents": 0},
            {"label": "Thu", "completed": 8, "incidents": 1},
            {"label": "Fri", "completed": 9, "incidents": 0},
        ],
        "monthlyMovement": [
            {"month": "Jan", "km": 118},
            {"month": "Feb", "km": 142},
            {"month": "Mar", "km": 131},
            {"month": "Apr", "km": 155},
            {"month": "May", "km": 162},
            {"month": "Jun", "km": 174},
        ],
        "batterySeries": [
            {"time": "08:00", "voltage": 48.2, "power": 182},
            {"time": "09:00", "voltage": 47.8, "power": 176},
            {"time": "10:00", "voltage": 47.4, "power": 171},
            {"time": "11:00", "voltage": 46.9, "power": 168},
            {"time": "12:00", "voltage": 47.1, "power": 170},
            {"time": "13:00", "voltage": 46.7, "power": 166},
            {"time": "14:00", "voltage": 46.3, "power": 164},
        ],
        "batteryStatus": {
            "packVoltage": 46.9,
            "packCurrent": 38.2,
            "stateOfCharge": 78,
            "temperature": "32°C",
            "cycles": 412,
            "health": "Good",
            "cells": [
                {"id": "Cell A", "voltage": 3.9},
                {"id": "Cell B", "voltage": 3.9},
                {"id": "Cell C", "voltage": 3.88},
                {"id": "Cell D", "voltage": 3.87},
            ],
        },
        "turns": {
            "left": 132,
            "right": 148,
        },
    }

    return jsonify(data)
