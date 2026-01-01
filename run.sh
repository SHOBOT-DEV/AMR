#!/bin/bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_DIR="$ROOT_DIR/backend"
FRONTEND_DIR="$ROOT_DIR/FrontEnd"
ROS2_WS="${ROS2_WS:-$ROOT_DIR/SHOBOT_AMR_ws}"
ROS2_DISTRO="${ROS2_DISTRO:-humble}"
ROS2_ENABLE="${ROS2_ENABLE:-0}"
ROS2_LAUNCH="${ROS2_LAUNCH:-shobot_navigation shobot_robot_bringup.py}"
ROS2_LOG="${ROS2_LOG:-$ROOT_DIR/ros2_launch.log}"
FLASK_PORT="${FLASK_PORT:-5000}"
FLASK_HOST="${FLASK_HOST:-127.0.0.1}"
FLASK_DEBUG="${FLASK_DEBUG:-1}"
INSTALL_FRONTEND_DEPS="${INSTALL_FRONTEND_DEPS:-auto}"
BACKEND_WAIT_RETRIES="${BACKEND_WAIT_RETRIES:-40}"
BACKEND_WAIT_DELAY="${BACKEND_WAIT_DELAY:-0.5}"

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Error: '$1' is required but not installed or not on PATH." >&2
    exit 1
  fi
}

require_directory() {
  if [[ ! -d "$1" ]]; then
    echo "Error: Expected directory '$1' does not exist." >&2
    exit 1
  fi
}

cleanup() {
  local exit_code=$?
  if [[ -n "${BACKEND_PID:-}" ]] && kill -0 "$BACKEND_PID" 2>/dev/null; then
    echo "Stopping Flask backend (PID $BACKEND_PID)..."
    kill "$BACKEND_PID" 2>/dev/null || true
    wait "$BACKEND_PID" 2>/dev/null || true
  fi
  if [[ -n "${ROS2_PID:-}" ]] && kill -0 "$ROS2_PID" 2>/dev/null; then
    echo "Stopping ROS2 launch (PID $ROS2_PID)..."
    kill "$ROS2_PID" 2>/dev/null || true
    wait "$ROS2_PID" 2>/dev/null || true
  fi
  exit "$exit_code"
}

trap cleanup EXIT INT TERM

wait_for_backend() {
  local attempt=1
  while (( attempt <= BACKEND_WAIT_RETRIES )); do
    if python - <<'PY'
import os
import socket

host = os.environ["FLASK_HOST"]
port = int(os.environ["FLASK_PORT"])

s = socket.socket()
s.settimeout(1)
try:
    s.connect((host, port))
except OSError:
    raise SystemExit(1)
else:
    s.close()
PY
    then
      echo "Flask backend is up at ${FLASK_HOST}:${FLASK_PORT}"
      return 0
    fi
    printf "Waiting for Flask backend (%s/%s)...\n" "$attempt" "$BACKEND_WAIT_RETRIES"
    sleep "$BACKEND_WAIT_DELAY"
    ((attempt++))
  done
  echo "Error: Flask backend did not become reachable at ${FLASK_HOST}:${FLASK_PORT}" >&2
  return 1
}

require_command python
require_command npm
require_directory "$BACKEND_DIR"
require_directory "$FRONTEND_DIR"

start_ros2_stack() {
  if [[ "$ROS2_ENABLE" != "1" ]]; then
    echo "ROS2 launch disabled (set ROS2_ENABLE=1 to start it)."
    return 0
  fi

  local ros_setup="/opt/ros/${ROS2_DISTRO}/setup.bash"
  if [[ ! -f "$ros_setup" ]]; then
    echo "Error: ROS2 distro '${ROS2_DISTRO}' not found at $ros_setup" >&2
    return 1
  fi

  echo "Sourcing ROS2: $ros_setup"
  # shellcheck disable=SC1090
  source "$ros_setup"

  local ws_setup="$ROS2_WS/install/setup.bash"
  if [[ -f "$ws_setup" ]]; then
    echo "Sourcing workspace overlay: $ws_setup"
    # shellcheck disable=SC1090
    source "$ws_setup"
  else
    echo "Warning: workspace overlay missing ($ws_setup). Build with 'colcon build' if needed."
  fi

  echo "Launching ROS2: ros2 launch ${ROS2_LAUNCH}"
  # shellcheck disable=SC2086
  ros2 launch ${ROS2_LAUNCH} >"$ROS2_LOG" 2>&1 &
  ROS2_PID=$!
  echo "ROS2 launch started (PID $ROS2_PID), logs: $ROS2_LOG"
}

start_ros2_stack

echo "Starting Flask backend on ${FLASK_HOST}:${FLASK_PORT}..."
cd "$BACKEND_DIR"
if [[ -f "/home/shahbaz/Project/Ongoing_project/Environment/AMR/bin/activate" ]]; then
  # shellcheck disable=SC1091
  source /home/shahbaz/Project/Ongoing_project/Environment/AMR/bin/activate
else
  echo "Error: Expected venv not found at /home/shahbaz/Project/Ongoing_project/Environment/AMR" >&2
  exit 1
fi
export FLASK_PORT FLASK_HOST FLASK_DEBUG
python run.py &
BACKEND_PID=$!
if [[ -n "${VIRTUAL_ENV:-}" && "$(type -t deactivate)" == "function" ]]; then
  deactivate
fi
cd "$ROOT_DIR"
wait_for_backend

echo "Installing frontend dependencies (if needed)..."
cd "$FRONTEND_DIR"
if [[ "$INSTALL_FRONTEND_DEPS" == "always" ]] || \
   { [[ "$INSTALL_FRONTEND_DEPS" != "never" ]] && [[ ! -d node_modules ]]; }; then
  npm install --legacy-peer-deps
else
  echo "Skipping npm install (INSTALL_FRONTEND_DEPS=$INSTALL_FRONTEND_DEPS)."
fi
if [[ ! -d node_modules ]]; then
  echo "Error: node_modules missing and INSTALL_FRONTEND_DEPS=never." >&2
  exit 1
fi
export REACT_APP_API_URL="${REACT_APP_API_URL:-http://127.0.0.1:$FLASK_PORT}"
echo "REACT_APP_API_URL=${REACT_APP_API_URL}"
echo "Starting React app (Ctrl+C to stop)..."
npm start
