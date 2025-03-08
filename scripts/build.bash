#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Ensure the project directory is set
if [ -z "$PROJECT_HOST_DIR" ]; then
    echo "❌ ERROR: Please source setup.bash first!"
    exit 1
fi

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "❌ ERROR: Docker daemon is not running. Please start Docker and try again."
    exit 1
fi

# Check if docker-compose.yml exists
if [ ! -f "$PROJECT_HOST_DIR/docker-compose.yml" ]; then
    echo "❌ ERROR: docker-compose.yml not found in $PROJECT_HOST_DIR!"
    exit 1
fi

# Move to project directory and build containers
echo "🔄 Building all Docker containers for project at $PROJECT_HOST_DIR..."
cd "$PROJECT_HOST_DIR"

export COMPOSE_BAKE=true

docker compose build

echo "✅ Docker containers built successfully!"
