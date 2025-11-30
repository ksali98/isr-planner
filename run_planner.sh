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

cd /Users/kamalali/isr_projects/isr_web || {
  echo "❌ Could not cd into /Users/kamalali/isr_projects/isr_web"
  exit 1
}

# Activate venv
source venv/bin/activate

# Make sure Python can see your project root
export PYTHONPATH=/Users/kamalali/isr_projects

# Run the FastAPI app
python -m uvicorn server.main:app --reload --port ${PLANNER_PORT}
