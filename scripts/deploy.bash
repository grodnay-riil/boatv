#!/bin/bash
RED='\e[31m';GREEN='\e[32m';YELLOW='\e[33m';BLUE='\e[34m';BOLD='\e[1m';NC='\e[0m' # No Color
# Ensure necessary environment variables are set
if [[ -z "$VERA_ROBOT_IP" || -z "$VERA_ROBOT_USER" || -z "$VERA_HOST_DIR" ]]; then
    echo -e "${RED}Error: Required environment variables (VERA_ROBOT_IP, VERA_ROBOT_USER, VERA_HOST_DIR) are not set.{$NC}"
    exit 1
fi

# Define the target ROS workspace on the Jetson
JETSON_ROS_WS="/home/$VERA_ROBOT_USER/$PROJECT_NAME"

# Print deployment details
echo -e "${YELLOW}$(date '+%H:%M:%S')${NC}"
echo -e "================= Starting Deployment to Jetson ================="
echo -e "Deploying from      : ${GREEN}${VERA_HOST_DIR}${NC}"
echo -e "Deploying to        : ${GREEN}${VERA_ROBOT_USER}@$VERA_ROBOT_IP:$JETSON_ROS_WS${NC}"
echo -e "========================================================"

# Change to the project host directory
cd "$VERA_HOST_DIR" || { echo -e "${RED}Failed to change directory to $VERA_HOST_DIR${NC}"; exit 1; }

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
    echo -e "Deployment to $VERA_ROBOT_IP was successful."
else
    echo -e "Deployment to $VERA_ROBOT_IP failed."
    exit 1
fi
