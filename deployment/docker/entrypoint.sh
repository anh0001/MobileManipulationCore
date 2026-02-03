#!/bin/bash
# Entrypoint script for Jetson container

set -e

# Source ROS 2
source /opt/ros/humble/setup.bash

# Source workspace if built
if [ -f "/workspace/install/setup.bash" ]; then
    source /workspace/install/setup.bash
fi

# Source Piper workspace if available
if [ -f "$HOME/piper_ws/install/setup.bash" ]; then
    source $HOME/piper_ws/install/setup.bash
fi

# Execute command
exec "$@"
