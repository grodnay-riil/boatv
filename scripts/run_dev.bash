#!/bin/bash

# Ensure the project directory is set
if [ -z "$PROJECT_HOST_DIR" ]; then
    echo "Please source setup.bash first!"
    exit 1
fi

echo "Running ros_build container interactively for project at ${PROJECT_HOST_DIR}..."
cd "$PROJECT_HOST_DIR"
docker compose run --rm dev
