# Use ROS_DIST to set the base image
ARG ROS_DIST=humble
FROM dockwater:${ROS_DIST}

# Set up environment variables
ARG PROJECT_NAME
ARG PROJECT_USER
ARG PROJECT_UID
ARG PROJECT_GID
ARG TARGET_ROS_WS

ENV PROJECT_NAME=${PROJECT_NAME}
ENV PROJECT_USER=${PROJECT_USER}
ENV PROJECT_UID=${PROJECT_UID}
ENV PROJECT_GID=${PROJECT_GID}
ENV ROS_WS=${TARGET_ROS_WS}
ENV DEBIAN_FRONTEND=noninteractive 

# Install cyclonedds and zenoh
RUN apt-get update && \
    apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp && \
    echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee -a /etc/apt/sources.list > /dev/null && \
    apt update && \
    apt install zenoh-bridge-ros2dds  \
    && rm -rf /var/lib/apt/lists/* \
    && apt clean -qq

# Install iperf3 for bandwidth testing
RUN apt-get update && apt-get install -y \
    iperf3 \          
    net-tools \       
    iputils-ping \    
    dnsutils \        
    iproute2 \
    nload \      
    && rm -rf /var/lib/apt/lists/* \
    && apt clean -qq


# Create a new user and configure sudo
RUN id -u $PROJECT_USER 2>/dev/null || \
    (groupadd -g $PROJECT_GID $PROJECT_USER && \
    useradd -m -u $PROJECT_UID -g $PROJECT_GID -s /bin/bash $PROJECT_USER && \
    usermod -aG sudo $PROJECT_USER) && \
    echo "$PROJECT_USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Enable bash completion and add a colorful prompt
RUN apt-get update && apt-get install -y bash-completion && \

echo "source /usr/share/bash-completion/bash_completion" >> /home/$PROJECT_USER/.bashrc && \
    echo 'PS1="\[\e[32m\]\u@\h:\[\e[34m\]\w\[\e[m\]\[\e[33m\]\$(__git_ps1)\[\e[m\]$ "' >> /home/$PROJECT_USER/.bashrc && \
    echo "#lines removed for autocomplete" > /etc/apt/apt.conf.d/docker-clean \
    && rm -rf /var/lib/apt/lists/* \
    && apt clean -qq


ENV HISTFILE=${ROS_WS}/.bash_history
ENV PROMPT_COMMAND='history -a'

# Switch to the new user
USER $PROJECT_USER
WORKDIR ${ROS_WS}

# # Run rosdep update
RUN rosdep update

# # Set up workspace and copy files
RUN mkdir -p /home/$PROJECT_USER/$PROJECT_NAME/src
COPY --chown=$PROJECT_UID:$PROJECT_GID . /home/$PROJECT_USER/$PROJECT_NAME

# # # Install dependencies and build workspace if src exists
RUN rosdep install --from-paths src --ignore-src -r -y
RUN rm -rf install build log && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --merge-install"; 

# Source workspace by default
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$PROJECT_USER/.bashrc && \
    echo 'if [ -f /home/$PROJECT_USER/$PROJECT_NAME/install/setup.bash ]; then source /home/$PROJECT_USER/$PROJECT_NAME/install/setup.bash; fi' >> /home/$PROJECT_USER/.bashrc && \
    echo "export PROMPT_COMMAND='history -a' && export HISTFILE=/home/$PROJECT_USER/$PROJECT_NAME/.bash_history" 
# Default command
CMD ["bash"]
