# Use ROS_DIST to set the base image
ARG ROS_DIST=humble
FROM osrf/ros:${ROS_DIST}-desktop-full

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

# Debug prints
RUN echo "Project Name: ${PROJECT_NAME}" && \
    echo "ROS_DISTRO is: ${ROS_DISTRO}"

# Install dependencies, tools, and bash-completion in a single RUN
RUN echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee -a /etc/apt/sources.list > /dev/null && \
    apt-get update && apt-get install -y --no-install-recommends \
    git git-extras bash-completion \
    python3-colcon-common-extensions python3-pip curl build-essential \
    iperf3 net-tools iputils-ping dnsutils iproute2 nload \
    zenoh-bridge-ros2dds \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    python3-rosdep && \
    rm -rf /var/lib/apt/lists/*

# Create a new user and configure sudo
RUN id -u $PROJECT_USER 2>/dev/null || \
    (groupadd -g $PROJECT_GID $PROJECT_USER && \
    useradd -m -u $PROJECT_UID -g $PROJECT_GID -s /bin/bash $PROJECT_USER && \
    usermod -aG sudo $PROJECT_USER) && \
    echo "$PROJECT_USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Enable bash completion and add a colorful prompt
RUN echo "source /usr/share/bash-completion/bash_completion" >> /home/$PROJECT_USER/.bashrc && \
    echo 'PS1="\[\e[32m\]\u@\h:\[\e[34m\]\w\[\e[m\]\[\e[33m\]\$(__git_ps1)\[\e[m\]$ "' >> /home/$PROJECT_USER/.bashrc 

# Modify APT configuration to enable package caching for autocompletion
RUN echo "#lines removed for autocomplete" > /etc/apt/apt.conf.d/docker-clean

RUN apt-get update && apt-get install -y --no-install-recommends vim
# Switch to the new user
USER $PROJECT_USER
WORKDIR /home/$PROJECT_USER

# Run rosdep update
RUN rosdep update

# Set up workspace and copy files
RUN mkdir -p /home/$PROJECT_USER/$PROJECT_NAME/src
WORKDIR /home/$PROJECT_USER/$PROJECT_NAME
COPY --chown=$PROJECT_UID:$PROJECT_GID . /home/$PROJECT_USER/$PROJECT_NAME

# Install dependencies and build workspace if src exists
RUN if [ -d "src" ]; then \
    rosdep install --from-paths src --ignore-src -r -y && \
    /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install"; \
    fi

# Source workspace by default
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$PROJECT_USER/.bashrc && \
    echo 'if [ -f /home/$PROJECT_USER/$PROJECT_NAME/install/setup.bash ]; then source /home/$PROJECT_USER/$PROJECT_NAME/install/setup.bash; fi' >> /home/$PROJECT_USER/.bashrc

# Default command
CMD ["bash"]
