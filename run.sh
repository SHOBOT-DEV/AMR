#!/bin/bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_DIR="$ROOT_DIR/backend"
FRONTEND_DIR="$ROOT_DIR/FrontEnd"
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

echo "Starting Flask backend on ${FLASK_HOST}:${FLASK_PORT}..."
cd "$BACKEND_DIR"
USING_BACKEND_VENV=0
if [[ -f "venv/bin/activate" ]]; then
  # shellcheck disable=SC1091
  source venv/bin/activate
  if python - <<'PY'
import importlib
required = ("flask", "flask_sqlalchemy", "flask_cors")
for pkg in required:
    importlib.import_module(pkg)
PY
  then
    USING_BACKEND_VENV=1
  else
    echo "Warning: backend venv missing dependencies; running with system Python instead."
    if [[ -n "${VIRTUAL_ENV:-}" && "$(type -t deactivate)" == "function" ]]; then
      deactivate
    fi
  fi
fi
export FLASK_PORT FLASK_HOST FLASK_DEBUG
python run.py &
BACKEND_PID=$!
if (( USING_BACKEND_VENV )) && [[ -n "${VIRTUAL_ENV:-}" && "$(type -t deactivate)" == "function" ]]; then
  deactivate
fi
cd "$ROOT_DIR"
wait_for_backend

echo "Installing frontend dependencies (if needed)..."
cd "$FRONTEND_DIR"
if [[ "$INSTALL_FRONTEND_DEPS" == "always" ]] || \
   { [[ "$INSTALL_FRONTEND_DEPS" != "never" ]] && [[ ! -d node_modules ]]; }; then
  npm install
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
