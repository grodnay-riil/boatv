#!/bin/bash

### --- VALIDATE ENVIRONMENT --- ###
# Ensure the project name is set
if [ -z "$VERA_PROJECT_NAME" ]; then
    echo "Please source setup.bash first!"
    exit 1
fi

### --- ATTACH TO RUNNING CONTAINER --- ###
cd "$VERA_HOST_DIR"
docker compose exec "${VERA_PROJECT_NAME}_dev" bash
