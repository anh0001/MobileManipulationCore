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
# Script to set up remote policy server
#
# This script prepares a remote GPU server for running VLA model inference

set -e

echo "========================================="
echo "Policy Server Setup"
echo "========================================="

# Check if running on a GPU-enabled machine
if ! command -v nvidia-smi &> /dev/null; then
    echo "WARNING: nvidia-smi not found. This server should have an NVIDIA GPU."
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Use a dedicated Python user base to avoid mixing with ROS 2 system Python.
PYTHONUSERBASE="${PYTHONUSERBASE:-$HOME/.local/ros2_humble}"
export PYTHONUSERBASE
export PATH="$PYTHONUSERBASE/bin:$PATH"

echo "Installing Python dependencies..."

# Upgrade pip (user base)
python3 -m pip install --upgrade --user pip

# Install PyTorch (CUDA-enabled)
# Adjust CUDA version as needed
python3 -m pip install --user torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Install Hugging Face transformers and related packages
python3 -m pip install --user transformers>=4.35.0
python3 -m pip install --user huggingface_hub
python3 -m pip install --user accelerate
python3 -m pip install --user sentencepiece
python3 -m pip install --user protobuf

# Install ROS 2 Python client (for ROS communication)
python3 -m pip install --user rclpy

# TODO: Install specific VLA model requirements
# python3 -m pip install --user lerobot
# python3 -m pip install --user openvla

echo ""
echo "========================================="
echo "Policy server setup complete!"
echo "========================================="
echo ""
echo "To start the policy server, ensure PYTHONUSERBASE is set, then run from the repo root:"
echo "  export PYTHONUSERBASE=\"$PYTHONUSERBASE\""
echo "  export PATH=\"\$PYTHONUSERBASE/bin:\$PATH\""
echo "  python3 -m manipulation_policy.policy_server"
echo ""
