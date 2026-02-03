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
# Script to update Piper to specific versions
#
# Usage: ./update_piper.sh [commit_hash]

set -e

PIPER_WS="$HOME/piper_ws"

if [ ! -d "$PIPER_WS/src/piper_ros" ]; then
    echo "ERROR: Piper workspace not found."
    echo "Please run install_piper.sh first."
    exit 1
fi

cd "$PIPER_WS/src/piper_ros"

if [ -n "$1" ]; then
    echo "Updating Piper to commit: $1"
    git fetch origin
    git checkout "$1"
else
    echo "Updating Piper to latest..."
    git pull
fi

echo "Rebuilding workspace..."
cd "$PIPER_WS"
source /opt/ros/humble/setup.bash
colcon build --symlink-install

echo "Piper updated successfully!"
