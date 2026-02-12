#!/usr/bin/env bash

PLANNER_PORT=8893

echo "🔍 Checking for existing planner on port ${PLANNER_PORT} ..."
# Find any process listening on the planner port and kill it
PIDS=$(lsof -ti:${PLANNER_PORT} -sTCP:LISTEN 2>/dev/null)

if [ -n "$PIDS" ]; then
  echo "🧹 Stopping old planner process(es): $PIDS"
  kill $PIDS 2>/dev/null || true
  # Give the OS a moment to free the port
  sleep 1
else
  echo "✅ No existing planner process found on port ${PLANNER_PORT}."
fi

echo "🚀 Starting ISR Planner on http://localhost:${PLANNER_PORT} ..."

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Change to the script directory (project root)
cd "$SCRIPT_DIR" || {
  echo "❌ Could not cd into $SCRIPT_DIR"
  exit 1
}

# Activate venv if it exists
if [ -d "venv" ]; then
  echo "📦 Activating virtual environment..."
  source venv/bin/activate
fi

# Make sure Python can see the project root
export PYTHONPATH="$SCRIPT_DIR"

# Run the FastAPI app
python3 -m uvicorn server.main:app --reload --port ${PLANNER_PORT}
