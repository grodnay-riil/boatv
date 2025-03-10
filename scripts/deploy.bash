#!/bin/bash

# Ensure necessary environment variables are set
if [[ -z "$ROBOT_IP" || -z "$ROBOT_USER" || -z "$PROJECT_HOST_DIR" ]]; then
    echo "Error: Required environment variables (ROBOT_IP, ROBOT_USER, PROJECT_HOST_DIR) are not set."
    exit 1
fi

# Define the target ROS workspace on the Jetson
JETSON_ROS_WS="/home/$ROBOT_USER/$PROJECT_NAME"

# Print deployment details
echo "================= Starting Deployment to Jetson ================="
echo "Deploying from      : $PROJECT_HOST_DIR"
echo "Deploying to        : $ROBOT_USER@$ROBOT_IP:$JETSON_ROS_WS"
echo "========================================================"

# Change to the project host directory
cd "$PROJECT_HOST_DIR" || { echo "Failed to change directory to $PROJECT_HOST_DIR"; exit 1; }

# Perform rsync with delete option, using the exclude list
rsync -av --delete -h --stats\
  --exclude='/devel/' \
  --exclude='/build/' \
  --exclude='/install/' \
  --exclude='/log/' \
  --exclude='.*' \
  ./ "$ROBOT_USER@$ROBOT_IP:$JETSON_ROS_WS"


# Check if rsync was successful
if [ $? -eq 0 ]; then
    echo "Deployment to $ROBOT_IP was successful."
else
    echo "Deployment to $ROBOT_IP failed."
    exit 1
fi
