# ISR Mission Planning Web Application
# Build: docker build -t isr-web .
# Run:   docker run -p 8893:8893 -e ANTHROPIC_API_KEY=your_key isr-web

FROM python:3.11-slim

# Set working directory
WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements first (for caching)
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy sibling packages (path_planning_core, isr_editor)
# These should be placed in the build context
COPY path_planning_core/ /app/path_planning_core/
COPY isr_editor/ /app/isr_editor/

# Copy application code
COPY server/ /app/isr_web/server/
COPY webapp/ /app/isr_web/webapp/
COPY docs/ /app/isr_web/docs/

# Set Python path to find all packages
ENV PYTHONPATH=/app

# Expose port
EXPOSE 8893

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD curl -f http://localhost:8893/ || exit 1

# Run the application
CMD ["python", "-m", "uvicorn", "isr_web.server.main:app", "--host", "0.0.0.0", "--port", "8893"]
