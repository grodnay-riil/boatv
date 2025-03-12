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
if docker compose up -d "dev"; then
    echo "-----------------------------------------"
    echo "Container '\e[32m${VERA_PROJECT_NAME}_dev'\e[0m started successfully!"
    echo "Next steps:"
    echo "➡️  Run '\e[32m./join_dev.bash\e[0m' to attach to the container."
    echo "➡️  Or use '\e[32mccd\e[0m' to navigate to the project directory."
    echo "➡️  Then run '\e[32mcode .\e[0m' to open the project in VS Code."
    echo "-----------------------------------------"
else
    echo "❌ Failed to start the container. Please check logs and try again."
fi
