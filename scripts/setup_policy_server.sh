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
# Script to set up remote policy server with conda (recommended)
#
# This script prepares a remote GPU server for running OpenVLA inference.
# For detailed instructions, see docs/openvla_remote.md

set -e

echo "========================================="
echo "Policy Server Setup (Conda Method)"
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

# Check for conda
if ! command -v conda &> /dev/null; then
    echo "ERROR: conda not found. Please install Miniconda or Anaconda first."
    echo "Visit: https://docs.conda.io/en/latest/miniconda.html"
    exit 1
fi

# Detect CUDA version
CUDA_VERSION="12.4"
if command -v nvidia-smi &> /dev/null; then
    DETECTED_CUDA=$(nvidia-smi | grep "CUDA Version" | sed -n 's/.*CUDA Version: \([0-9]*\.[0-9]*\).*/\1/p')
    if [[ -n "$DETECTED_CUDA" ]]; then
        echo "Detected CUDA version: $DETECTED_CUDA"
        # Map to supported PyTorch CUDA versions
        if [[ "$DETECTED_CUDA" == 11.* ]]; then
            CUDA_VERSION="11.8"
        elif [[ "$DETECTED_CUDA" == 12.* ]]; then
            CUDA_VERSION="12.4"
        fi
    fi
fi

echo "Using PyTorch CUDA version: $CUDA_VERSION"

# Conda environment name
ENV_NAME="mobile_manipulation_vla"

# Check if environment already exists
if conda env list | grep -q "^${ENV_NAME} "; then
    echo "Conda environment '$ENV_NAME' already exists."
    read -p "Remove and recreate? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        conda env remove -n "$ENV_NAME" -y
    else
        echo "Using existing environment."
    fi
fi

# Create conda environment if it doesn't exist
if ! conda env list | grep -q "^${ENV_NAME} "; then
    echo "Creating conda environment: $ENV_NAME"
    conda create -n "$ENV_NAME" python=3.10 -y
fi

# Activate environment
echo "Activating conda environment..."
eval "$(conda shell.bash hook)"
conda activate "$ENV_NAME"

# Install PyTorch
echo "Installing PyTorch with CUDA $CUDA_VERSION..."
if [[ "$CUDA_VERSION" == "11.8" ]]; then
    conda install pytorch torchvision torchaudio pytorch-cuda=11.8 -c pytorch -c nvidia -y
else
    conda install pytorch torchvision torchaudio pytorch-cuda=12.4 -c pytorch -c nvidia -y
fi

# Install pinned versions for stability
echo "Installing pinned dependency versions..."
pip install torch==2.2.0 torchvision==0.17.0
pip install transformers==4.40.1 tokenizers==0.19.1 timm==0.9.10

# Install repo requirements
echo "Installing repo requirements..."
if [[ -f "requirements.txt" ]]; then
    pip install -r requirements.txt
else
    echo "WARNING: requirements.txt not found in current directory"
fi

# Install Flash Attention 2
echo "Installing Flash Attention 2..."
pip install packaging ninja
pip install "flash-attn==2.5.5" --no-build-isolation

# Install HuggingFace CLI
echo "Installing HuggingFace CLI..."
pip install huggingface_hub

echo ""
echo "========================================="
echo "Policy server setup complete!"
echo "========================================="
echo ""
echo "Next steps:"
echo "1. Authenticate with HuggingFace:"
echo "   conda activate $ENV_NAME"
echo "   huggingface-cli login"
echo ""
echo "2. (Optional) Clone OpenVLA for development:"
echo "   cd ~/"
echo "   git clone https://github.com/openvla/openvla.git"
echo "   cd openvla"
echo "   pip install -e ."
echo ""
echo "3. Start the policy server:"
echo "   conda activate $ENV_NAME"
echo "   cd <repo-root>"
echo "   python3 -m manipulation_policy.policy_server --host 0.0.0.0 --port 30542"
echo ""
echo "For detailed instructions, see docs/openvla_remote.md"
echo ""
