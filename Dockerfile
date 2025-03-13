# Use VERA_ROS_DIST to set the base image; dummy default requires override from Compose.
ARG VERA_ROS_DIST=dummy
# FROM dockwater:${VERA_ROS_DIST}
FROM osrf/ros:${VERA_ROS_DIST}-desktop
# Set up environment variables
ARG VERA_ROS_DIST
ARG VERA_PROJECT_NAME
ARG VERA_DOCKER_USER
ARG VERA_DOCKER_UID
ARG VERA_DOCKER_GID
ARG VERA_DOCKER_DIR

ENV VERA_PROJECT_NAME=${VERA_PROJECT_NAME}
ENV VERA_DOCKER_USER=${VERA_DOCKER_USER}
ENV VERA_DOCKER_UID=${VERA_DOCKER_UID}
ENV VERA_DOCKER_GID=${VERA_DOCKER_GID}
ENV VERA_ROS_DIST=${VERA_ROS_DIST}
ENV ROS_WS=${VERA_DOCKER_DIR}
ENV DEBIAN_FRONTEND=noninteractive 

ARG APT_PROXY=http://172.17.0.1:3142 #This is typically the IP of the host machine during build
# Attempt to use apt-cacher-ng; if unavailable, continue without proxy.
RUN if curl -fsSL --connect-timeout 2 "${APT_PROXY}/acng-report.html" >/dev/null; then \
      echo -e "\033[32m Using apt-cacher-ng proxy at ${APT_PROXY}\033[0m"  >&2; \
      echo "Acquire::http::Proxy \"${APT_PROXY}\";" > /etc/apt/apt.conf.d/01proxy; \
    else \
      echo "\033[33m apt-cacher-ng unavailable at ${APT_PROXY}, using direct internet.\033[0m"  >&2; \
    fi
# Install some useful tools
RUN apt-get update && apt-get install -y iperf3 net-tools iputils-ping dnsutils iproute2 nload \    
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean -qq

# Create a new user and configure sudo
RUN groupadd -g $VERA_DOCKER_GID $VERA_DOCKER_USER && \
    useradd -m -u $VERA_DOCKER_UID -g $VERA_DOCKER_GID -G sudo,dialout -s /bin/bash $VERA_DOCKER_USER && \
    echo "$VERA_DOCKER_USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Enable bash completion and add a colorful prompt
RUN apt-get update && apt-get install -y bash-completion && \
    echo "source /usr/share/bash-completion/bash_completion" >> /home/$VERA_DOCKER_USER/.bashrc && \
    echo 'PS1="\[\e[32m\]\u@\h:\[\e[34m\]\w\[\e[m\]\[\e[33m\]\$(__git_ps1)\[\e[m\]$ "' >> /home/$VERA_DOCKER_USER/.bashrc && \
    echo "#lines removed for autocomplete" > /etc/apt/apt.conf.d/docker-clean \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean -qq
ENV HISTFILE=${VERA_DOCKER_DIR}/.bash_history
ENV PROMPT_COMMAND='history -a'

COPY --chown=$VERA_DOCKER_UID:$VERA_DOCKER_GID . /home/$VERA_DOCKER_USER/$VERA_PROJECT_NAME

# Set up directories and permissions (-p flag ensure it does not overite existing directories)
RUN mkdir -p /home/$VERA_DOCKER_USER/$VERA_PROJECT_NAME/src && \
    mkdir -p /home/$VERA_DOCKER_USER/.vscode-server && \
    mkdir -p /home/$VERA_DOCKER_USER/.gz && \
    mkdir -p /home/$VERA_DOCKER_USER/.ssh && \
    chown -R $VERA_DOCKER_UID:$VERA_DOCKER_GID  /home/$VERA_DOCKER_USER/.gz /home/$VERA_DOCKER_USER/.vscode-server /home/$VERA_DOCKER_USER/.ssh

#Were done with root, switch to user
USER $VERA_DOCKER_USER
WORKDIR ${VERA_DOCKER_DIR}
# Install dependencies and build workspace if src exists
# (We do not clean after apt-get update to keep the cache)
RUN rosdep update --rosdistro=${VERA_ROS_DIST} && sudo apt-get update && rosdep install --from-paths src --ignore-src -r -y
RUN /bin/bash -c "source /opt/ros/${VERA_ROS_DIST}/setup.bash && colcon build --base-paths src --symlink-install"; 

# Source workspace by default
RUN echo "source /opt/ros/${VERA_ROS_DIST}/setup.bash" >> /home/$VERA_DOCKER_USER/.bashrc && \
    echo 'if [ -f /home/$VERA_DOCKER_USER/$VERA_PROJECT_NAME/install/setup.bash ]; then source /home/$VERA_DOCKER_USER/$VERA_PROJECT_NAME/install/setup.bash; else echo "⚠️ setup.bash not found at ~/install. Did you build the workspace? (colcon build --symlink-install)"; fi' >> /home/$VERA_DOCKER_USER/.bashrc && \
    echo "alias ccd='cd $VERA_DOCKER_DIR'" >> ~/.bashrc && \
    echo "export PROMPT_COMMAND='history -a' && export HISTFILE=/home/$VERA_DOCKER_USER/$VERA_PROJECT_NAME/.bash_history" && \
    echo "source $VERA_DOCKER_DIR/scripts/setup.bash >> /home/$VERA_DOCKER_USER/.bashrc"
# Default command
CMD ["bash"]