#!/usr/bin/env bash
set -e

PYTHON_CMD=${PYTHON_CMD:-python3}
ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="$ROOT_DIR/.venv"

echo "Using Python: $($PYTHON_CMD --version)"

if [[ ! -d "$VENV_DIR" ]]; then
  echo "Creating virtual environment..."
  $PYTHON_CMD -m venv "$VENV_DIR"
else
  echo "Virtual environment already exists."
fi

# Activate
# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

# Upgrade pip
pip install --upgrade pip

# Install dependencies
if [[ -f "$ROOT_DIR/requirements.txt" ]]; then
  echo "Installing backend requirements..."
  pip install -r "$ROOT_DIR/requirements.txt"
else
  echo "❌ requirements.txt not found!"
  exit 1
fi

echo "✅ Virtual environment ready at $VENV_DIR"
echo "To activate manually: source .venv/bin/activate"





# #!/usr/bin/env bash
# set -e

# # Creates a Python virtual environment in .venv, activates it, and installs requirements.
# # Usage: bash setup_venv.sh
# PYTHON_CMD=${PYTHON_CMD:-python3}

# # create venv
# $PYTHON_CMD -m venv .venv

# # activate (for informational output only; user must source for current shell)
# echo "Virtual environment created at .venv"
# echo "To activate (Linux/macOS): source .venv/bin/activate"
# echo "To activate (Windows PowerShell): .venv\\Scripts\\Activate.ps1"
# echo "After activation you can install packages with: pip install -r requirements.txt"

# # install requirements if file exists
# if [ -f requirements.txt ]; then
#   echo "Installing requirements..."
#   . .venv/bin/activate
#   pip install -r requirements.txt
#   echo "Requirements installed."
# else
#   echo "No requirements.txt found — skipping pip install."
# fi


