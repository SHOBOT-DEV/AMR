#!/usr/bin/env bash
set -e

# Creates a Python virtual environment in .venv, activates it, and installs requirements.
# Usage: bash setup_venv.sh
PYTHON_CMD=${PYTHON_CMD:-python3}

# create venv
$PYTHON_CMD -m venv .venv

# activate (for informational output only; user must source for current shell)
echo "Virtual environment created at .venv"
echo "To activate (Linux/macOS): source .venv/bin/activate"
echo "To activate (Windows PowerShell): .venv\\Scripts\\Activate.ps1"
echo "After activation you can install packages with: pip install -r requirements.txt"

# install requirements if file exists
if [ -f requirements.txt ]; then
  echo "Installing requirements..."
  . .venv/bin/activate
  pip install -r requirements.txt
  echo "Requirements installed."
else
  echo "No requirements.txt found — skipping pip install."
fi
