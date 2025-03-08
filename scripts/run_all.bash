#!/bin/bash

# Check if PROJECT_DIR is set
if [ -z "$PROJECT_DIR" ]; then
    echo "❌ PROJECT_DIR is not set! Please export it and try again."
    exit 1
fi

# Move to the project directory
cd "$PROJECT_DIR" || { echo "❌ Failed to change to $PROJECT_DIR!"; exit 1; }

# Define the tmux session name based on the project directory
SESSION_NAME=$(basename "$PROJECT_DIR")

# Check if the tmux session already exists and kill it if it does
tmux has-session -t "$SESSION_NAME" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "⚠️  Tmux session $SESSION_NAME already exists. Killing it..."
    tmux kill-session -t "$SESSION_NAME"
fi

# Start existing containers without building
echo "🚀 Starting existing containers without building..."
docker compose up -d

# Check if the containers started successfully
if [ $? -ne 0 ]; then
    echo "❌ Failed to start containers!"
    exit 1
fi


# Create a new tmux session with a single window and split it into 4 panes
echo "🖥  Creating new tmux session with 4 panes: $SESSION_NAME"
tmux new-session -d -t "$SESSION_NAME" 

# Bind Ctrl-b x to run kill_all.bash
tmux bind-key x run-shell "$PROJECT_DIR/scripts/kill_all.bash"


tmux send-keys "docker compose attach ros_publisher" C-m
tmux split "docker compose attach ros_subscriber" 
tmux split "docker compose exec -it ros_publisher bash"
tmux split "docker compose exec -it ros_subscriber bash"
tmux select-layout  tiled  # Arrange panes neatly

# Enable mouse mode for easy switching
tmux setw  mouse on

# Attach to the tmux session
tmux attach -t "$SESSION_NAME"
