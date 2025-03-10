#!/bin/bash

# Determine the script's directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_HOST_DIR="$(dirname "$SCRIPT_DIR")"

# Force set project name based on the directory name
export PROJECT_NAME="$(basename "$PROJECT_HOST_DIR")"

# Force set user and ROS distribution
export PROJECT_USER="${PROJECT_NAME}user"
export ROS_DIST="humble"  # options are only humble
export ROS_DOMAIN_ID="1"

# Set project directory environment variable
export PROJECT_HOST_DIR="$PROJECT_HOST_DIR"

# Get current user ID and group ID and force set them
export PROJECT_UID=$(id -u)
export PROJECT_GID=$(id -g)

# Add the scripts directory to the PATH forcefully
export PATH="$SCRIPT_DIR:$PATH"

# Set the target ROS workspace in docker
export TARGET_ROS_WS="/home/$PROJECT_USER/$PROJECT_NAME"

# Set the robot IP and user
export ROBOT_IP=10.42.0.2
export ROBOT_USER=forecr

# Print configuration details
echo "================= ROS Docker Setup ================="
echo "Robot IP            : $ROBOT_IP"
echo "Robot User          : $ROBOT_USER"
echo "Project Name        : $PROJECT_NAME"
echo "Project User        : $PROJECT_USER (UID: $PROJECT_UID, GID: $PROJECT_GID)"
echo "Project Host Dir    : $PROJECT_HOST_DIR"
echo "Target ROS Workspace: $TARGET_ROS_WS"
echo "ROS Distribution    : $ROS_DIST"
echo "ROS Domain ID       : $ROS_DOMAIN_ID"
echo "Scripts in PATH     : $SCRIPT_DIR"
echo "===================================================="
