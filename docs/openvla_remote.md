# OpenVLA Remote Server Guide

This guide explains how to run OpenVLA on a remote GPU server, connect the robot-side policy node to it, and validate the end-to-end path.

The repo ships a lightweight HTTP server in `src/manipulation_policy/manipulation_policy/policy_server.py`. By default it returns a stub response. Replace `build_stub_response()` with OpenVLA inference to produce real actions.

## Data Contract

### Request JSON (robot -> server)

The policy node sends JSON to `POST /infer` with these fields:

- `reference_frame` (string, optional)
- `joint_states` (object)
- `joint_states.name` (string array)
- `joint_states.position` (float array)
- `joint_states.velocity` (float array)
- `joint_states.effort` (float array)
- `image` (base64-encoded JPEG string, optional)
- `image_encoding` (string, optional, usually `jpeg`)
- `image_width` (int, optional)
- `image_height` (int, optional)

### Response JSON (server -> robot)

Return JSON that maps to `manipulation_msgs/PolicyOutput.msg`:

- `has_eef_target` (bool)
- `eef_target_pose.position` (object with `x`, `y`, `z` floats)
- `eef_target_pose.orientation` (object with `x`, `y`, `z`, `w` floats)
- `has_joint_deltas` (bool)
- `joint_deltas` (float array)
- `gripper_command` (float)
- `gripper_active` (bool)
- `has_base_hint` (bool)
- `base_velocity_hint.linear` (object with `x`, `y`, `z` floats)
- `base_velocity_hint.angular` (object with `x`, `y`, `z` floats)
- `reference_frame` (string)
- `confidence` (float)

Example response:

```json
{
  "has_eef_target": true,
  "eef_target_pose": {
    "position": {"x": 0.3, "y": 0.0, "z": 0.3},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
  },
  "has_joint_deltas": false,
  "joint_deltas": [],
  "gripper_command": 0.5,
  "gripper_active": true,
  "has_base_hint": false,
  "base_velocity_hint": {
    "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
  },
  "reference_frame": "base_link",
  "confidence": 0.0
}
```

## Remote GPU Server Setup

You have two options for setting up the remote policy server:

### Automated Setup (Easiest)

For a quick, automated setup using conda, run the provided script:

```bash
cd ~/MobileManipulationCore
bash scripts/setup_policy_server.sh
```

This script will:
- Check for GPU and conda availability
- Auto-detect CUDA version
- Create the `mobile_manipulation_vla` conda environment
- Install all dependencies with pinned versions
- Install Flash Attention 2
- Provide next steps for authentication and running the server

After running the script, skip to the "Authenticate with HuggingFace" section below.

### Manual Setup

If you prefer manual control or need to customize the installation, follow one of these options:

#### Option A: Conda Environment (Recommended)

**Best for:** Remote GPU servers, clean isolation, easier dependency management

1. Clone the repo on the GPU server:

```bash
git clone <your-repo-url> ~/MobileManipulationCore
cd ~/MobileManipulationCore
git submodule update --init --recursive
```

2. Create and activate conda environment:

```bash
conda create -n mobile_manipulation_vla python=3.10 -y
conda activate mobile_manipulation_vla
```

3. Install PyTorch for your CUDA version:

```bash
# For CUDA 12.4
conda install pytorch torchvision torchaudio pytorch-cuda=12.4 -c pytorch -c nvidia -y

# For CUDA 11.8
# conda install pytorch torchvision torchaudio pytorch-cuda=11.8 -c pytorch -c nvidia -y
```

4. Install repo requirements:

```bash
pip install -r requirements.txt
```

5. Continue to "Official OpenVLA Installation" section below.

#### Option B: System Python with PYTHONUSERBASE

**Best for:** Environments where conda is unavailable or when ROS 2 is installed system-wide

1. Clone the repo on the GPU server:

```bash
git clone <your-repo-url> ~/MobileManipulationCore
cd ~/MobileManipulationCore
git submodule update --init --recursive
```

2. Configure a dedicated Python user base:

```bash
export PYTHONUSERBASE="$HOME/.local/mobile_manipulation"
export PATH="$PYTHONUSERBASE/bin:$PATH"

# Persist for future shells
echo 'export PYTHONUSERBASE="$HOME/.local/mobile_manipulation"' >> ~/.bashrc
echo 'export PATH="$PYTHONUSERBASE/bin:$PATH"' >> ~/.bashrc

python3 -m pip install --upgrade --user pip
```

3. Install dependencies:

```bash
# Install PyTorch for your CUDA version
python3 -m pip install --user torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Install repo requirements
python3 -m pip install --user -r requirements.txt
```

4. Continue to "Official OpenVLA Installation" section below.

## Official OpenVLA Installation

Based on the [official OpenVLA repository](https://github.com/openvla/openvla).

### System Requirements

- Python 3.10+
- PyTorch 2.2.*
- CUDA-capable GPU

**Tested and recommended versions:**
- PyTorch 2.2.0
- torchvision 0.17.0
- transformers 4.40.1
- tokenizers 0.19.1
- timm 0.9.10
- flash-attn 2.5.5

> **Important:** Later versions of transformers, timm, and tokenizers have reported regressions and breaking changes. The versions above are explicitly pinned for stability.

### Installation Steps

#### Conda Installation (Recommended)

If you followed **Option A: Conda Environment** above, your environment is already activated. Now install OpenVLA:

```bash
# Ensure your conda environment is active
conda activate mobile_manipulation_vla

# Install pinned versions for stability
pip install torch==2.2.0 torchvision==0.17.0
pip install transformers==4.40.1 tokenizers==0.19.1 timm==0.9.10

# Clone and install OpenVLA
cd ~/
git clone https://github.com/openvla/openvla.git
cd openvla
pip install -e .

# Install Flash Attention 2 (required for optimal performance)
pip install packaging ninja
ninja --version  # Verify ninja installation
pip install "flash-attn==2.5.5" --no-build-isolation
```

#### System Python Installation (PYTHONUSERBASE)

If you followed **Option B: System Python with PYTHONUSERBASE** above:

```bash
# Ensure PYTHONUSERBASE is set
export PYTHONUSERBASE="$HOME/.local/mobile_manipulation"
export PATH="$PYTHONUSERBASE/bin:$PATH"

# Install pinned versions for stability
python3 -m pip install --user torch==2.2.0 torchvision==0.17.0
python3 -m pip install --user transformers==4.40.1 tokenizers==0.19.1 timm==0.9.10

# Install minimal requirements
python3 -m pip install --user -r https://raw.githubusercontent.com/openvla/openvla/main/requirements-min.txt

# Install Flash Attention 2
python3 -m pip install --user packaging ninja
python3 -m pip install --user "flash-attn==2.5.5" --no-build-isolation
```

### Authenticate with HuggingFace

Set your HuggingFace token to download the pretrained models:

```bash
# Install HuggingFace CLI (if not already installed)
pip install huggingface_hub

# Login with your token (recommended - persists credentials)
huggingface-cli login
# Paste your token when prompted

# Alternative: set as environment variable
export HUGGINGFACE_HUB_TOKEN="your_token_here"
```

### Model Licensing

**Important:** While the OpenVLA code uses an MIT License, the pretrained models inherit restrictions from their base models. Both OpenVLA models derive from Llama-2 and are subject to the [Llama Community License](https://ai.meta.com/llama/license/).

### Verify Installation

Test that OpenVLA loads correctly:

```bash
# If using conda
conda activate mobile_manipulation_vla

# Test loading the model
python3 << 'EOF'
from transformers import AutoModelForVision2Seq, AutoProcessor
import torch

print("Loading OpenVLA model...")
processor = AutoProcessor.from_pretrained("openvla/openvla-7b", trust_remote_code=True)
model = AutoModelForVision2Seq.from_pretrained(
    "openvla/openvla-7b",
    torch_dtype=torch.bfloat16,
    low_cpu_mem_usage=True,
    trust_remote_code=True
)
print("✓ OpenVLA loaded successfully!")
print(f"Model device: {model.device}")
print(f"Model dtype: {model.dtype}")
EOF
```

## OpenVLA Inference Implementation

The policy server now includes OpenVLA inference implementation in `src/manipulation_policy/manipulation_policy/policy_server.py`. The `build_stub_response()` function:

- Decodes base64-encoded images from requests
- Builds prompts from task instructions and joint states
- Runs OpenVLA inference with Flash Attention 2
- Converts 7-DOF actions (x, y, z, roll, pitch, yaw, gripper) to PolicyOutput JSON
- Returns no-op responses when images are missing

The implementation supports:
- Automatic model loading with thread-safe singleton pattern
- GPU/CPU device selection via `OPENVLA_DEVICE` environment variable
- Custom model selection via `OPENVLA_MODEL_ID` (default: `openvla/openvla-7b`)
- Flash Attention 2 or fallback attention via `OPENVLA_ATTENTION_IMPL`
- Action unnormalization via `OPENVLA_UNNORM_KEY` (default: `bridge_orig`)

## Run the Remote Server

Start the server and bind it to all interfaces so the robot can reach it.

### If Using Conda:

```bash
# Activate conda environment
conda activate mobile_manipulation_vla

# Navigate to repo root
cd ~/MobileManipulationCore

# Set environment variables (optional)
export OPENVLA_MODEL_ID="openvla/openvla-7b"  # or openvla/openvla-1b
export OPENVLA_DEVICE="cuda"  # or cuda:0, cuda:1, etc.
export OPENVLA_ATTENTION_IMPL="flash_attention_2"
export OPENVLA_UNNORM_KEY="bridge_orig"

# Start the server
python3 -m manipulation_policy.policy_server --host 0.0.0.0 --port 30542
```

### If Using System Python with PYTHONUSERBASE:

```bash
# Set PYTHONUSERBASE
export PYTHONUSERBASE="$HOME/.local/mobile_manipulation"
export PATH="$PYTHONUSERBASE/bin:$PATH"

# Navigate to repo root
cd ~/MobileManipulationCore

# Set environment variables (optional)
export OPENVLA_MODEL_ID="openvla/openvla-7b"
export OPENVLA_DEVICE="cuda"

# Start the server
python3 -m manipulation_policy.policy_server --host 0.0.0.0 --port 30542
```

The server exposes:

- `GET /health` - Health check endpoint
- `POST /infer` - Policy inference endpoint

On first request, the server will download and cache the OpenVLA model (~14GB for 7B variant).

## Configure the Robot Side

Launch the manipulation stack with remote inference enabled:

```bash
ros2 launch manipulation_bringup core_launch.py \
  use_remote_policy:=true \
  remote_url:=http://YOUR_SERVER_IP:30542
```

Optional ROS 2 parameters to tune the remote path:

- `remote_timeout_sec`
- `remote_retry_attempts`
- `remote_fallback_on_failure`

## Test the Remote Server

1. Health check:

```bash
curl http://YOUR_SERVER_IP:30542/health
```

Expected response:

```json
{"status": "ok"}
```

2. Inference request with minimal payload (no image):

```bash
curl -X POST http://YOUR_SERVER_IP:30542/infer \
  -H 'Content-Type: application/json' \
  -d '{
    "reference_frame": "base_link",
    "joint_states": {"name": [], "position": [], "velocity": [], "effort": []}
  }'
```

If the server is wired to OpenVLA, you should receive a valid `PolicyOutput`-shaped JSON response.

3. End-to-end ROS test on the robot:

```bash
ros2 topic echo /manipulation/policy_output
```

## Troubleshooting

- `connection refused`: check the server host, port, and firewall rules.
- `timeout`: increase `remote_timeout_sec` or verify server load.
- `invalid json`: ensure the request body is valid JSON.
- `out of memory`: use a smaller model variant or enable FP16.
