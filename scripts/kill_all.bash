#!/bin/bash

# Check if VERA_HOST_DIR is set
if [ -z "$VERA_HOST_DIR" ]; then
    echo "❌ VERA_HOST_DIR is not set! Please export it and try again."
    exit 1
fi

# Move to the project directory
cd "$VERA_HOST_DIR" || { echo "❌ Failed to change to $VERA_HOST_DIR!"; exit 1; }

# Define the tmux session name based on the project directory
SESSION_NAME=$(basename "$VERA_HOST_DIR")

# Stop and remove all containers for the project
echo "🛑 Stopping and removing all containers for project $SESSION_NAME..."
docker compose down --volumes --remove-orphans
#docker compoose kill
# Check if the tmux session exists and kill it if it does
tmux has-session -t "$SESSION_NAME" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "🛑 Killing tmux session: $SESSION_NAME..."
    tmux kill-session -t "$SESSION_NAME"
else
    echo "ℹ️  No tmux session named $SESSION_NAME found."
fi

echo "✅ All containers and tmux sessions for $SESSION_NAME have been stopped."
