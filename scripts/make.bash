if ! ([ -f /.dockerenv ]); then
    echo -e "\033[31mError: This script must be run inside a Docker container. Exiting.\033[0m"
    exit 1
fi

# Ensure the script is run inside the ROS 2 workspace
if [ -z "$VERA_PROJECT_NAME" ] || [ -z "$VERA_DOCKER_DIR" ]; then
    echo "❌ ERROR: Environment variables not set!"
    echo "➡️  Run 'source setup.bash' first."
    exit 1
fi

# Navigate to workspace
cd "$VERA_DOCKER_DIR"

# Parse command-line arguments
BUILD_SRC_ONLY=true
if [[ "$1" == "--src-only" ]]; then
    BUILD_SRC_ONLY=true
fi

source /opt/ros/$VERA_ROS_DIST/setup.bash

# Run colcon build
if [ "$BUILD_SRC_ONLY" = true ]; then
    echo "🚀 Starting colcon build (Only src/ packages)..."
    colcon build --symlink-install --base-paths src/
else
    echo "🚀 Starting full colcon build..."
    colcon build --symlink-install
fi

# Check if build was successful
if [ $? -eq 0 ]; then
    echo "✅ Colcon build completed successfully!"
else
    echo "❌ Colcon build failed!"
    exit 1
fi

# Source workspace setup file
echo "🎯 Colcon build is complete." 
echo "🔗 Now source new workspace setup (or use "smake")..."
