#!/bin/bash
# Copyright 2026 MobileManipulationCore Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Script to install or update Piper ROS driver and SDK
#
# This script fetches the required Piper dependencies at known-good versions
# specified in config/piper_versions.yaml

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
CONFIG_FILE="$PROJECT_ROOT/config/piper_versions.yaml"

echo "========================================="
echo "Piper Installation Script"
echo "========================================="

# Check if config file exists
if [ ! -f "$CONFIG_FILE" ]; then
    echo "ERROR: Configuration file not found: $CONFIG_FILE"
    echo "Please ensure config/piper_versions.yaml exists"
    exit 1
fi

# Parse version information from YAML
# (This is a simple parser - for production, consider using yq or python)
PIPER_ROS_REPO=$(grep "piper_ros_repo:" "$CONFIG_FILE" | awk '{print $2}' || echo "https://github.com/agilexrobotics/piper_ros.git")
PIPER_ROS_BRANCH=$(grep "piper_ros_branch:" "$CONFIG_FILE" | awk '{print $2}' || echo "humble")
PIPER_ROS_COMMIT=$(grep "piper_ros_commit:" "$CONFIG_FILE" | awk '{print $2}' || echo "")
PIPER_SDK_VERSION=$(grep "piper_sdk_version:" "$CONFIG_FILE" | awk '{print $2}' || echo "latest")

echo "Configuration:"
echo "  Piper ROS Repo: $PIPER_ROS_REPO"
echo "  Piper ROS Branch: $PIPER_ROS_BRANCH"
echo "  Piper ROS Commit: ${PIPER_ROS_COMMIT:-latest}"
echo "  Piper SDK Version: $PIPER_SDK_VERSION"
echo ""

# Create workspace if it doesn't exist
PIPER_WS="$HOME/piper_ws"
if [ ! -d "$PIPER_WS" ]; then
    echo "Creating Piper workspace at: $PIPER_WS"
    mkdir -p "$PIPER_WS/src"
fi

cd "$PIPER_WS/src"

# Clone or update Piper ROS repository
if [ -d "piper_ros" ]; then
    echo "Updating existing piper_ros repository..."
    cd piper_ros
    git fetch origin
    if [ -n "$PIPER_ROS_COMMIT" ]; then
        git checkout "$PIPER_ROS_COMMIT"
    else
        git checkout "$PIPER_ROS_BRANCH"
        git pull
    fi
else
    echo "Cloning piper_ros repository..."
    git clone -b "$PIPER_ROS_BRANCH" "$PIPER_ROS_REPO"
    cd piper_ros
    if [ -n "$PIPER_ROS_COMMIT" ]; then
        git checkout "$PIPER_ROS_COMMIT"
    fi
fi

echo ""
echo "Installing Piper SDK..."

# Install Piper SDK via pip
if [ "$PIPER_SDK_VERSION" == "latest" ]; then
    pip3 install piper_sdk --upgrade
else
    pip3 install piper_sdk=="$PIPER_SDK_VERSION"
fi

echo ""
echo "Installing ROS dependencies..."
cd "$PIPER_WS"
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo ""
echo "Building Piper workspace..."
colcon build --symlink-install

echo ""
echo "========================================="
echo "Piper installation complete!"
echo "========================================="
echo ""
echo "To use Piper in your terminal, run:"
echo "  source $PIPER_WS/install/setup.bash"
echo ""
echo "Or add this line to your ~/.bashrc:"
echo "  echo 'source $PIPER_WS/install/setup.bash' >> ~/.bashrc"
echo ""
