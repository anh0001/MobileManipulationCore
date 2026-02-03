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

# Create virtual environment if it doesn't exist
VENV_PATH="$HOME/policy_venv"
if [ ! -d "$VENV_PATH" ]; then
    echo "Creating Python virtual environment..."
    python3 -m venv "$VENV_PATH"
fi

# Activate virtual environment
source "$VENV_PATH/bin/activate"

echo "Installing Python dependencies..."

# Upgrade pip
pip install --upgrade pip

# Install PyTorch (CUDA-enabled)
# Adjust CUDA version as needed
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Install Hugging Face transformers and related packages
pip install transformers>=4.35.0
pip install huggingface_hub
pip install accelerate
pip install sentencepiece
pip install protobuf

# Install ROS 2 Python client (for ROS communication)
pip install rclpy

# TODO: Install specific VLA model requirements
# pip install lerobot
# pip install openvla

echo ""
echo "========================================="
echo "Policy server setup complete!"
echo "========================================="
echo ""
echo "To activate the environment, run:"
echo "  source $VENV_PATH/bin/activate"
echo ""
echo "To start the policy server, run:"
echo "  # TODO: Add server startup command"
echo ""
