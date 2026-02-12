# OpenVLA Remote Server Guide

This guide explains how to run OpenVLA on a remote GPU server, connect the robot-side policy node to it, and validate the end-to-end path.

The repo ships a lightweight HTTP server in `src/manipulation_policy/manipulation_policy/policy_server.py`. By default it returns a stub response. Replace `build_stub_response()` with OpenVLA inference to produce real actions.

## Data Contract

### Request JSON (robot -> server)

The policy node sends JSON to `POST /infer` with these fields:

- `reference_frame` (string, optional)
- `openvla_xyz_scaling` (float, optional; gain for OpenVLA xyz deltas, default `1.0`)
- `openvla_rotation_scaling` (float, optional; gain for OpenVLA roll/pitch/yaw deltas, default `1.0`)
- `openvla_clip_actions` (bool, optional; clamp scaled OpenVLA outputs, default `false`)
- `openvla_position_bounds` (float[2], optional; clamp bounds for xyz deltas)
- `openvla_rotation_bounds` (float[2], optional; clamp bounds for roll/pitch/yaw deltas)
- `openvla_gripper_bounds` (float[2], optional; clamp bounds for raw gripper output before normalization)
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
- `gripper_command` (float, normalized in `[0.0, 1.0]`; `0.0` = closed, `1.0` = open)
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

### Gripper Semantics

`gripper_command` is a normalized command, not a raw joint position. The adapter maps it to the configured
gripper joint range as:

`target = closed_position + gripper_command * (open_position - closed_position)`

For the default Piper config in this repo, `closed_position = 0.0` and `open_position = 0.035`, so:
- `gripper_command = 1.0` maps to `0.035` (fully open)
- `gripper_command = 0.0` maps to `0.0` (fully closed)
- `gripper_command = 0.996` maps to `0.03486` (effectively fully open)

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
- Builds prompts from task instructions
- Runs OpenVLA inference with Flash Attention 2
- Converts 7-DOF actions (x, y, z, roll, pitch, yaw, gripper) to PolicyOutput JSON
- Returns no-op responses when images are missing

The implementation supports:
- Automatic model loading with thread-safe singleton pattern
- GPU/CPU device selection via `OPENVLA_DEVICE` environment variable
- Custom model selection via `OPENVLA_MODEL_ID` (default: `openvla/openvla-7b`)
- Flash Attention 2 or fallback attention via `OPENVLA_ATTENTION_IMPL`
- Optional action unnormalization via `OPENVLA_UNNORM_KEY` (unset by default in this repo)
- Action gains via request fields `openvla_xyz_scaling` / `openvla_rotation_scaling`
- Env fallbacks `OPENVLA_XYZ_SCALING` / `OPENVLA_ROTATION_SCALING`
- Optional output clamping via request fields `openvla_clip_actions`, `openvla_position_bounds`,
  `openvla_rotation_bounds`, `openvla_gripper_bounds`
- Env fallbacks `OPENVLA_CLIP_ACTIONS`, `OPENVLA_POSITION_BOUNDS`,
  `OPENVLA_ROTATION_BOUNDS`, `OPENVLA_GRIPPER_BOUNDS`
- Optional joint-state prompt context via `OPENVLA_INCLUDE_JOINT_STATES_IN_PROMPT` (default: `false`)

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
# Optional: only set this if your model checkpoint expects a specific unnorm key
# export OPENVLA_UNNORM_KEY="your_unnorm_key"
export OPENVLA_XYZ_SCALING="1.0"  # fallback xyz gain if request does not include openvla_xyz_scaling
export OPENVLA_ROTATION_SCALING="1.0"  # fallback rotation gain if request does not include openvla_rotation_scaling
export OPENVLA_CLIP_ACTIONS="false"  # fallback clamp toggle if request does not include openvla_clip_actions
export OPENVLA_POSITION_BOUNDS="-1.0,1.0"  # fallback clamp bounds for xyz deltas
export OPENVLA_ROTATION_BOUNDS="-3.141592653589793,3.141592653589793"  # fallback clamp bounds for roll/pitch/yaw deltas
export OPENVLA_GRIPPER_BOUNDS="0.0,1.0"  # fallback clamp bounds for raw gripper output
# Optional: append joint states to prompt text (usually keep this disabled)
export OPENVLA_INCLUDE_JOINT_STATES_IN_PROMPT="false"

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

Safety note:
- Keep `remote_fallback_on_failure` set to `false` in remote mode so the robot does not execute stub/default actions when the OpenVLA server is unavailable or times out.

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
