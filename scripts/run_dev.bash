#!/bin/bash

### --- VALIDATE ENVIRONMENT --- ###
# Ensure the project directory is set
if [ -z "$VERA_PROJECT_NAME" ]; then
    echo "Please source setup.bash first!"
    exit 1
fi

### --- RUN DOCKER CONTAINER --- ###
xhost +
echo "Running ros_build container interactively for project at ${VERA_HOST_DIR}..."
cd "$VERA_HOST_DIR"

# Start the container
if docker compose up --rm -d "${VERA_PROJECT_NAME}_dev"; then
    echo "-----------------------------------------"
    echo "Container '${VERA_PROJECT_NAME}_dev' started successfully!"
    echo "Next steps:"
    echo "➡️  Run './join_dev.bash' to attach to the container."
    echo "➡️  Or use 'ccd' to navigate to the project directory."
    echo "➡️  Then run 'code .' to open the project in VS Code."
    echo "-----------------------------------------"
else
    echo "❌ Failed to start the container. Please check logs and try again."
fi
