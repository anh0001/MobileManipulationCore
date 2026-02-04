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

1. Clone the repo on the GPU server.

2. Configure a dedicated Python user base (keeps pip packages isolated from ROS 2 system Python).
   If you plan to use the conda workflow below and run the server inside that conda env, you can skip this step.

```bash
cd ~/MobileManipulationCore
export PYTHONUSERBASE="$HOME/.local/ros2_humble"
export PATH="$PYTHONUSERBASE/bin:$PATH"
# Persist for future shells if desired
echo 'export PYTHONUSERBASE="$HOME/.local/ros2_humble"' >> ~/.bashrc
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
If you are using the conda workflow below, run these inside the conda environment and omit `--user`.

4. Install OpenVLA following the official installation guide.

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

#### Option 1: Full Installation (with conda)

```bash
# Create and activate conda environment
conda create -n openvla python=3.10 -y
conda activate openvla

# Install PyTorch (adjust CUDA version as needed - example uses CUDA 12.4)
conda install pytorch torchvision torchaudio pytorch-cuda=12.4 -c pytorch -c nvidia -y

# Clone and install OpenVLA
git clone https://github.com/openvla/openvla.git
cd openvla
pip install -e .

# Install Flash Attention 2 (required for optimal performance)
pip install packaging ninja
ninja --version  # Verify ninja installation
pip install "flash-attn==2.5.5" --no-build-isolation
```

#### Option 2: Minimal Installation (inference only)

For running inference only without training dependencies:

```bash
# Install minimal requirements
python3 -m pip install --user -r https://raw.githubusercontent.com/openvla/openvla/main/requirements-min.txt

# Install Flash Attention 2
python3 -m pip install --user packaging ninja
python3 -m pip install --user "flash-attn==2.5.5" --no-build-isolation
```

### Download Model Weights

Set your HuggingFace token to download the pretrained models:

```bash
export HUGGINGFACE_HUB_TOKEN="your_token_here"
```

Or use `huggingface-cli login` for persistent authentication.

### Model Licensing

**Important:** While the OpenVLA code uses an MIT License, the pretrained models inherit restrictions from their base models. Both OpenVLA models derive from Llama-2 and are subject to the [Llama Community License](https://ai.meta.com/llama/license/).

### Verify Installation

```python
from transformers import AutoModelForVision2Seq, AutoProcessor
import torch

# Test loading the model
processor = AutoProcessor.from_pretrained("openvla/openvla-7b", trust_remote_code=True)
model = AutoModelForVision2Seq.from_pretrained(
    "openvla/openvla-7b",
    torch_dtype=torch.bfloat16,
    low_cpu_mem_usage=True,
    trust_remote_code=True
)
print("OpenVLA loaded successfully!")
```

## Add OpenVLA Inference to the Server

Edit `src/manipulation_policy/manipulation_policy/policy_server.py` and replace `build_stub_response()` with OpenVLA inference. The function receives the request JSON and must return a dict matching the response schema above.

Minimum checklist for the OpenVLA implementation:

- Decode `request["image"]` from base64 JPEG.
- Use the decoded image and joint states to build model inputs.
- Run the model on the GPU in FP16 when possible.
- Convert the model output into the `PolicyOutput`-compatible JSON.

## Run the Remote Server

Start the server and bind it to all interfaces so the robot can reach it:

```bash
export PYTHONUSERBASE="$HOME/.local/ros2_humble"
export PATH="$PYTHONUSERBASE/bin:$PATH"
python3 -m manipulation_policy.policy_server --host 0.0.0.0 --port 5000
```

The server exposes:

- `GET /health`
- `POST /infer`

## Configure the Robot Side

Launch the manipulation stack with remote inference enabled:

```bash
ros2 launch manipulation_bringup core_launch.py \
  use_remote_policy:=true \
  remote_url:=http://YOUR_SERVER_IP:5000
```

Optional ROS 2 parameters to tune the remote path:

- `remote_timeout_sec`
- `remote_retry_attempts`
- `remote_fallback_on_failure`

## Test the Remote Server

1. Health check:

```bash
curl http://YOUR_SERVER_IP:5000/health
```

Expected response:

```json
{"status": "ok"}
```

2. Inference request with minimal payload (no image):

```bash
curl -X POST http://YOUR_SERVER_IP:5000/infer \
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
