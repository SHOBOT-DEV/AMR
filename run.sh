#!/bin/bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_DIR="$ROOT_DIR/backend"
FRONTEND_DIR="$ROOT_DIR/FrontEnd"
FLASK_PORT="${FLASK_PORT:-5000}"

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

echo "Starting Flask backend on port $FLASK_PORT..."
cd "$BACKEND_DIR"
if [[ -f "venv/bin/activate" ]]; then
  # shellcheck disable=SC1091
  source venv/bin/activate
fi
FLASK_RUN_PORT="$FLASK_PORT" python run.py &
BACKEND_PID=$!
if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  deactivate
fi
cd "$ROOT_DIR"

echo "Installing frontend dependencies (if needed)..."
cd "$FRONTEND_DIR"
if [[ ! -d node_modules ]]; then
  npm install
fi
export REACT_APP_API_URL="${REACT_APP_API_URL:-http://127.0.0.1:$FLASK_PORT}"
echo "REACT_APP_API_URL=${REACT_APP_API_URL}"
echo "Starting React app (Ctrl+C to stop)..."
npm start
