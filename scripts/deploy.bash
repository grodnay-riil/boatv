#!/bin/bash

# Ensure necessary environment variables are set
if [[ -z "$VERA_ROBOT_IP" || -z "$VERA_ROBOT_USER" || -z "$VERA_HOST_DIR" ]]; then
    echo "Error: Required environment variables (VERA_ROBOT_IP, VERA_ROBOT_USER, VERA_HOST_DIR) are not set."
    exit 1
fi

# Define the target ROS workspace on the Jetson
JETSON_ROS_WS="/home/$VERA_ROBOT_USER/$PROJECT_NAME"

# Print deployment details
echo "================= Starting Deployment to Jetson ================="
echo "Deploying from      : $VERA_HOST_DIR"
echo "Deploying to        : $VERA_ROBOT_USER@$VERA_ROBOT_IP:$JETSON_ROS_WS"
echo "========================================================"

# Change to the project host directory
cd "$VERA_HOST_DIR" || { echo "Failed to change directory to $VERA_HOST_DIR"; exit 1; }

# Perform rsync with delete option, using the exclude list
rsync -i -av --delete -h \
  --exclude='/devel/' \
  --exclude='/build/' \
  --exclude='/install/' \
  --exclude='/log/' \
  --exclude='.*' \
  ./ "$VERA_ROBOT_USER@$VERA_ROBOT_IP:$VERA_ROBOT_DIR/"


# Check if rsync was successful
if [ $? -eq 0 ]; then
    echo "Deployment to $VERA_ROBOT_IP was successful."
else
    echo "Deployment to $VERA_ROBOT_IP failed."
    exit 1
fi
