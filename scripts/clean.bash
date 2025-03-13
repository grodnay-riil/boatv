#!/bin/bash
#Remove colcon build artifacts
echo "Cleaning concon build artifacts..."
cd "$VERA_HOST_DIR"
rm -rf build install log
echo "Done!"