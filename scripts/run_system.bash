#!/bin/bash

# Check if VERA_HOST_DIR is set
if [ -z "$VERA_HOST_DIR" ]; then
    echo "❌ VERA_HOST_DIR is not set! source scripts/setup.bash."
    exit 1
fi

# Move to the project directory
cd "$VERA_HOST_DIR" || { echo "❌ Failed to change to $VERA_HOST_DIR!"; exit 1; }

# Define the tmux session name based on the project directory
SESSION_NAME=$(basename "$VERA_PROJECT_NAME")

# Check if the tmux session already exists and kill it if it does
tmux has-session -t "$SESSION_NAME" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "⚠️  Tmux session $SESSION_NAME already exists. Killing it..."
    tmux kill-session -t "$SESSION_NAME"
fi

# Start existing containers without building
echo "🚀 Starting existing containers home and robot..."
run_dev.bash
ssrobot -t "source ${VERA_ROBOT_DIR}/scripts/setup.bash && run_dev.bash"

# Create a new tmux session with a single window and split it into 4 panes
echo "🖥  Creating new tmux session with 4 panes: $SESSION_NAME"
tmux new-session -d -t "$SESSION_NAME" 

# Bind Ctrl-b x to run kill_all.bash
tmux bind-key x run-shell "kill_all.bash && ssrobot -c kill_all.bash"

tmux send-keys "docker compose up ui" C-m
tmux split "ssrobot -t 'cd ${VERA_ROBOT_DIR} && source scripts/setup.bash && docker compose up hw'"
tmux select-layout  tiled  # Arrange panes neatly

# Enable mouse mode for easy switching
tmux setw  mouse on

# Attach to the tmux session
tmux attach -t "$SESSION_NAME"
