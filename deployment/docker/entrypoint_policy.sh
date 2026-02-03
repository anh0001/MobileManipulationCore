#!/bin/bash
# Entrypoint script for policy server container

set -e

# Source ROS 2 if available
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Set Python path
export PYTHONPATH=/app:$PYTHONPATH

# Execute command
exec "$@"
