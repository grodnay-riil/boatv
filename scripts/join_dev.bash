#!/bin/bash

# Ensure the project directory is set
if [ -z "$PROJECT_DIR" ]; then
    echo "Please source setup.bash first!"
    exit 1
fi

echo "Checking if the ros_dev container is running..."

# Check if the ros_dev container is running
if ! docker ps --filter "name=ros_dev" --format "{{.Names}}" | grep -q "ros_dev"; then
    echo "ros_dev container is not running. Starting it..."
    docker compose run --rm -d ros_dev
    echo "ros_dev container started."
else
    echo "ros_dev container is already running."
fi

# Join the ros_dev container
echo "Joining the ros_dev container..."
docker compose exec ros_dev bash
