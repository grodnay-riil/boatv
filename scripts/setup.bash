#!/bin/bash

### --- GENERAL CONFIGURATION --- ###
export VERA_MODEL=${VERA_MODEL:-sbg}
export VERA_ROS_DIST="humble"
export ROS_LICENSE="© 2024 Skana Robotics Ltd. All rights reserved."
export VERA_DEFAULT_DOMAIN_ID=$(ip -4 addr show | grep -oP '(?<=inet\s)10\.42\.0\.\d+' >/dev/null && echo $VERA_HOST_DOMAIN_ID || echo $VERA_ROBOT_DOMAIN_ID)



### --- HOST CONFIGURATION --- ###
export VERA_HOST_IP=10.42.0.2
export VERA_HOST_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PATH="$VERA_HOST_SCRIPT_DIR:$PATH"
export VERA_HOST_DIR="$(dirname "$VERA_HOST_SCRIPT_DIR")"
export VERA_PROJECT_NAME="$(basename "$VERA_HOST_DIR")"
export VERA_HOST_DOMAIN_ID="1"

### --- DOCKER CONFIGURATION --- ###
export VERA_DOCKER_USER="${VERA_PROJECT_NAME}user"
export VERA_DOCKER_UID=$(id -u)
export VERA_DOCKER_GID=$(id -g)
export VERA_DOCKER_DIR="/home/$VERA_DOCKER_USER/$VERA_PROJECT_NAME"

### --- ROBOT CONFIGURATION --- ###
export VERA_ROBOT_IP=10.42.2.2
export VERA_ROBOT_USER=forecr
export VERA_ROBOT_DIR="/home/$VERA_ROBOT_USER/$VERA_PROJECT_NAME"
export VERA_ROBOT_DOMAIN_ID="2"

### --- ALIASES --- ###
alias ccd="cd $VERA_HOST_DIR"
alias ssrobot="ssh $VERA_ROBOT_USER@$VERA_ROBOT_IP"
alias ss="source $VERA_DOCKER_DIR/install/setup.bash"
alias smake="make.bash && echo 🔗 Sourcing for you... && source $VERA_DOCKER_DIR/install/setup.bash" 
alias autodeploy="watch -c -n 2 deploy.bash"
source $VERA_HOST_SCRIPT_DIR/autobackground.bash
### --- FINAL PROMPT / CONFIRMATION --- ###
echo -e "\n=========================================================================="
echo -e "\e[1mVERA®\e[0m Versitile Engine for Robotic Architecture  on \e[32m${VERA_ROS_DIST}@domain_id=${ROS_DOMAIN_ID}\e[0m is ready to go!"
echo -e "Host  \e[32m$VERA_HOST_IP:$VERA_HOST_DIR($(git branch --show-current))\e[0m"
echo -e "Robot: \e[32m$VERA_ROBOT_USER@$VERA_ROBOT_IP:$VERA_ROBOT_DIR\e[0m"
echo -e "=========================================================================="
echo -e "Next steps: 📦 build.bash | 🚀 run_dev.bash | 🖥️ join_dev.bash | 🔨 make.bash \n"

