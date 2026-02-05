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

### Delta Actions

OpenVLA emits **delta** end-effector motions (not absolute poses). The robot-side adapter can interpret these
as deltas when `action.eef_pose_is_delta: true` in `config/policy_params.yaml` (passed to the adapter as
`eef_target_is_delta`). If you provide absolute targets instead, set this flag to `false`.

## Remote GPU Server Setup

### Prerequisites

- CUDA-capable GPU (tested with NVIDIA RTX A6000, CUDA 12.6)
- Conda or Miniconda installed
- Python 3.10+

### Installation Steps

1. **Clone the repository:**

```bash
git clone <your-repo-url> ~/MobileManipulationCore
cd ~/MobileManipulationCore
git submodule update --init --recursive
```

2. **Create and activate conda environment:**

```bash
conda create -n mobile_manipulation_vla python=3.10 -y
conda activate mobile_manipulation_vla
```

3. **Install PyTorch with CUDA support:**

```bash
# For CUDA 12.x (adjust pytorch-cuda version as needed)
conda install pytorch==2.2.0 torchvision==0.17.0 torchaudio pytorch-cuda=12.1 -c pytorch -c nvidia -y
```

4. **Install repository requirements:**

```bash
pip install -r requirements.txt
```

5. **Install pinned versions for OpenVLA stability:**

```bash
pip install transformers==4.40.1 tokenizers==0.19.1 timm==0.9.10
```

> **Important:** These specific versions are required for stability. Later versions have reported regressions and breaking changes.

6. **Install the manipulation_policy package:**

```bash
# Navigate to the manipulation_policy package directory
cd ~/MobileManipulationCore/src/manipulation_policy

# Install the package in editable mode
pip install -e .
```

> **Important:** This step is required for Python to find the `manipulation_policy` module when running the policy server.

7. **Clone and install OpenVLA:**

```bash
cd ~/
git clone https://github.com/openvla/openvla.git
cd openvla
pip install -e .
```

8. **Install Flash Attention 2 (required for optimal performance):**

```bash
pip install packaging ninja

# Install pre-built wheel for faster installation
pip install https://github.com/Dao-AILab/flash-attention/releases/download/v2.5.5/flash_attn-2.5.5+cu122torch2.2cxx11abiFALSE-cp310-cp310-linux_x86_64.whl
```

### Authenticate with HuggingFace

```bash
huggingface-cli login
# Paste your HuggingFace token when prompted
```

Get your token from [https://huggingface.co/settings/tokens](https://huggingface.co/settings/tokens)

### Model Licensing

**Important:** While the OpenVLA code uses an MIT License, the pretrained models inherit restrictions from their base models. Both OpenVLA models derive from Llama-2 and are subject to the [Llama Community License](https://ai.meta.com/llama/license/).

### Verify Installation

Test that OpenVLA loads correctly with GPU:

```bash
conda activate mobile_manipulation_vla

python3 << 'EOF'
from transformers import AutoModelForVision2Seq, AutoProcessor
import torch

print("Testing OpenVLA with GPU...")
print(f"CUDA available: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"CUDA device: {torch.cuda.get_device_name(0)}")

processor = AutoProcessor.from_pretrained("openvla/openvla-7b", trust_remote_code=True)
model = AutoModelForVision2Seq.from_pretrained(
    "openvla/openvla-7b",
    torch_dtype=torch.bfloat16,
    low_cpu_mem_usage=True,
    trust_remote_code=True
).to("cuda")

print(f"\n✓ OpenVLA loaded successfully!")
print(f"Model device: {model.device}")
print(f"Model dtype: {model.dtype}")
EOF
```

Expected output:
```
CUDA available: True
CUDA device: NVIDIA RTX A6000
✓ OpenVLA loaded successfully!
Model device: cuda:0
Model dtype: torch.bfloat16
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

Start the server and bind it to all interfaces so the robot can reach it:

```bash
# Activate conda environment
eval "$(${HOME}/miniconda3/bin/conda shell.bash hook)" && conda activate mobile_manipulation_vla

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
