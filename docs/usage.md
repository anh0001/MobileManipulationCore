# Usage Guide

This guide explains how to install, configure, run, and extend the MobileManipulationCore stack.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Installation](#installation)
3. [Configuration](#configuration)
4. [Running the Stack](#running-the-stack)
5. [Monitoring and Debugging](#monitoring-and-debugging)
6. [Extending the System](#extending-the-system)

## Prerequisites

### Hardware Requirements

**Minimum (Jetson-only deployment):**
- NVIDIA Jetson AGX Orin (32GB or 64GB recommended)
- Mobile manipulator robot with:
  - RGB camera (RGB-D preferred)
  - Robotic arm (6+ DOF)
  - Mobile base with odometry
  - Gripper

**Recommended (split deployment):**
- Same robot as above
- Remote GPU server with NVIDIA GPU (RTX 3090, A100, or better)
- Reliable network connection between robot and server

### Software Requirements

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.10+
- CUDA 11.8+ (for Jetson and GPU server)
- Git

## Installation

### Step 1: Install ROS 2 Humble

If not already installed:

```bash
# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistall/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Setup environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Clone the Repository

```bash
cd ~/
mkdir -p manipulation_ws/src
cd manipulation_ws/src
git clone <repository-url> MobileManipulationCore
cd MobileManipulationCore
git submodule update --init --recursive
cd ../..
```

### Step 3: Install Dependencies

```bash
# Install ROS dependencies
sudo apt install python3-colcon-common-extensions python3-rosdep
sudo rosdep init  # Skip if already initialized
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Install Python ML dependencies (use PYTHONUSERBASE to avoid ROS 2 system conflicts)
cd src/MobileManipulationCore
export PYTHONUSERBASE="$HOME/.local/mobile_manipulation_core"
export PATH="$PYTHONUSERBASE/bin:$PATH"
# Persist for future shells if desired
echo 'export PYTHONUSERBASE="$HOME/.local/mobile_manipulation_core"' >> ~/.bashrc
echo 'export PATH="$PYTHONUSERBASE/bin:$PATH"' >> ~/.bashrc

python3 -m pip install --upgrade --user pip
# Install PyTorch (choose the correct CUDA/CPU wheel for your platform)
python3 -m pip install --user torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
# Install remaining dependencies
python3 -m pip install --user -r requirements.txt

# Optional: ROS 2 <-> NumPy helpers (if needed by your integration)
# The ros2_numpy submodule is a ROS 2 package and will be built by colcon.
```

### Step 4: Install Piper Dependencies (if using Piper arm)

```bash
cd src/MobileManipulationCore
./scripts/install_piper.sh
```

Follow the prompts. This will:
- Clone the Piper ROS repository
- Install Piper SDK
- Build the Piper workspace

Add to your `.bashrc`:
```bash
echo "source ~/piper_ws/install/setup.bash" >> ~/.bashrc
```

### Step 5: Build the Workspace

```bash
cd ~/manipulation_ws
colcon build --symlink-install

# Source the workspace
source install/setup.bash
echo "source ~/manipulation_ws/install/setup.bash" >> ~/.bashrc
```

## Configuration

### Robot Parameters

Edit `config/robot_params.yaml` to match your robot:

```yaml
robot:
  frames:
    base_link: "your_base_link"
    ee_link: "your_ee_link"
    camera_link: "your_camera_link"
  topics:
    camera_rgb: "/your/camera/topic"
    joint_states: "/your/joint_states"
    # ... etc
```

### Policy Configuration

Edit `config/policy_params.yaml`:

```yaml
policy:
  model_name: "openvla-7b"  # or your preferred model
  inference_rate: 10.0  # Adjust based on compute capability
  action_scaling_xyz: 1.0  # Gain for OpenVLA xyz delta outputs
  action_scaling_rotation: 1.0  # Gain for OpenVLA roll/pitch/yaw delta outputs
  clip_actions: true  # Clamp scaled OpenVLA outputs before execution
  action_bounds:
    position: [-0.10, 0.10]  # Clamp xyz deltas (m)
    rotation: [-0.35, 0.35]  # Clamp roll/pitch/yaw deltas (rad)
    gripper: [0.0, 1.0]  # Clamp raw OpenVLA gripper output
  use_fp16: true  # Enable for Jetson
  use_tensorrt: false  # Enable after optimization
```

### Piper Versions

If you need specific Piper versions, edit `config/piper_versions.yaml`:

```yaml
piper_ros_commit: "abc123def"  # Specific commit hash
piper_sdk_version: "1.2.3"
```

## Running the Stack

### On Real Robot (Jetson-Only)

1. **Start the base stack** (navigation + hardware drivers):
   ```bash
   # This should be provided by your base repository
   ros2 launch ranger_garden_assistant nav_stack.launch.py
   ```

2. **Start manipulation stack**:
   ```bash
   ros2 launch manipulation_bringup core_launch.py
   ```

3. **Verify nodes are running**:
   ```bash
   ros2 node list
   # Should see:
   # /perception_node
   # /policy_node
   # /adapter_node
   ```

4. **Send interactive task prompts**:
   ```bash
   ros2 run manipulation_policy task_prompt_cli
   ```
   Then type natural-language prompts in the CLI:
   ```text
   > pick up the bottle
   > place it in the bin
   > stop
   ```
   Notes:
   - Prompt topic defaults to `/manipulation/task_prompt` (configurable via `policy.task_prompt_topic`).
   - `policy.max_steps` controls how many inference steps execute per prompt.
   - Set `policy.max_steps <= 0` for continuous execution until `stop`/`clear`.

### In Simulation

```bash
ros2 launch manipulation_bringup sim_launch.py
```

### Using the Ranger Garden Assistant Base Stack

If the robot base stack is installed in `/home/robofi/codes/ranger-garden-assistant`, make sure to
build and source it before launching this repo so ROS can resolve the packages and TF tree:

```bash
# Build the base stack (one-time or after changes)
cd /home/robofi/codes/ranger-garden-assistant
colcon build --symlink-install

# Source the base stack
source /home/robofi/codes/ranger-garden-assistant/install/setup.bash

# Source this stack (overlay)
source ~/manipulation_ws/install/setup.bash
```

Launch the base stack:
```bash
ros2 launch robofi_bringup ranger_complete_bringup.launch.py
```

Then launch the manipulation stack:
```bash
ros2 launch manipulation_bringup core_launch.py
```

**Notes:**
- The Tier IV C2-176 camera publishes `/camera/image_raw` by default.
- The expected camera optical frame is `camera_optical_frame`.
- If your PiPER arm uses a different end-effector frame, update `config/robot_params.yaml`.

You can also launch both stacks together (requires `robofi_bringup` to be in your ROS environment):
```bash
ros2 launch manipulation_bringup ranger_integration.launch.py
```

This will:
- Launch Gazebo (when configured)
- Start all manipulation components
- Use simulation sensor topics

### With Remote Policy Server

For OpenVLA-specific remote deployment instructions (server setup, inference wiring, and test commands),
see `docs/openvla_remote.md`.

1. **On GPU server**, set up environment:
   ```bash
   cd ~/MobileManipulationCore
   ./scripts/setup_policy_server.sh
   export PYTHONUSERBASE="$HOME/.local/mobile_manipulation_core"
   export PATH="$PYTHONUSERBASE/bin:$PATH"
   ```

2. **Start policy server**:
   ```bash
   ros2 launch manipulation_policy remote_server_launch.py
   ```
   If ROS 2 CLI is not installed on the server, you can run it directly:
   ```bash
   python3 -m manipulation_policy.policy_server
   ```
   By default the server listens on `0.0.0.0:5000` and exposes:
   - `GET /health`
   - `POST /infer`

   You can override host/port:
   ```bash
   ros2 launch manipulation_policy remote_server_launch.py host:=0.0.0.0 port:=5000
   ```

3. **On Jetson**, start with remote mode:
   ```bash
   ros2 launch manipulation_bringup core_launch.py \
     use_remote_policy:=true \
     remote_url:=http://YOUR_SERVER_IP:5000
   ```

## Monitoring and Debugging

### Check Topics

```bash
# List all topics
ros2 topic list

# Monitor policy output
ros2 topic echo /manipulation/policy_output

# Monitor base commands
ros2 topic echo /cmd_vel

# View camera feed
ros2 run rqt_image_view rqt_image_view
```

### Check TF Tree

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo base_link camera_link
```

### Visualization with RViz

```bash
ros2 run rviz2 rviz2
```

Add displays:
- RobotModel
- TF
- Camera
- Markers (for debugging)

### Logs

Check node logs:
```bash
# During runtime
ros2 node info /policy_node

# Log files (if using launch with --log-level)
ls ~/.ros/log/
```

### Common Issues

**Issue:** Policy node crashes with CUDA out of memory
- **Solution:** Reduce model size, enable FP16, or use remote inference

**Issue:** Adapter not receiving policy output
- **Solution:** Check topic names match, verify policy node is running

**Issue:** TF lookup fails
- **Solution:** Ensure all required TF frames are being published, check frame names

**Issue:** Arm not moving
- **Solution:** Verify Piper driver is running, check action server connection

## Extending the System

### Adding a New Sensor

1. Create subscriber in `manipulation_perception`:
   ```cpp
   sensor_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
     "/scan", 10,
     std::bind(&PerceptionNode::sensorCallback, this, std::placeholders::_1));
   ```

2. Update observation structure to include new data

3. Modify policy input preprocessing if needed

### Using a Different VLA Model

1. Update `policy_node.py`:
   ```python
   def load_model(self):
       from transformers import AutoModel
       self.model = AutoModel.from_pretrained("your-model-name")
       return self.model
   ```

2. Adjust `config/policy_params.yaml`:
   ```yaml
   policy:
     model_name: "your-model-name"
     # Add model-specific parameters
   ```

3. Update inference logic in `run_inference()` method

### Adding Safety Constraints

1. Edit `adapter_node.cpp`:
   ```cpp
   bool AdapterNode::checkSafety(const PolicyOutput& action) {
       // Add your safety checks
       if (action.eef_target_pose.position.z < 0.0) {
           RCLCPP_WARN(this->get_logger(), "Target below ground!");
           return false;
       }
       return true;
   }
   ```

2. Call in policy callback before execution

### Custom Launch Configuration

Create `custom_launch.py` in your own package:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('manipulation_bringup'),
            '/launch/core_launch.py'
        ]),
        launch_arguments={
            'camera_topic': '/my_custom_camera'
        }.items()
    )

    return LaunchDescription([core])
```

## Performance Tuning

### Optimize Model Inference

**For Jetson:**
```bash
# Convert model to TensorRT
# (Requires additional setup - see NVIDIA docs)
```

**Enable FP16:**
```yaml
# config/policy_params.yaml
policy:
  use_fp16: true
```

### Adjust Control Frequency

```yaml
# config/policy_params.yaml
policy:
  inference_rate: 5.0  # Lower if compute-limited
```

```yaml
# robot params for adapter
adapter:
  control_rate: 50.0  # Higher for smoother control
```

### Network Optimization (Split Deployment)

- Use image compression
- Reduce image resolution
- Tune DDS QoS settings

## Testing

### Unit Tests

```bash
colcon test --packages-select manipulation_adapter
colcon test-result --verbose
```

### Integration Test

```bash
# Launch with test scenario
ros2 launch manipulation_bringup test_launch.py
```

### Hardware Test Checklist

- [ ] Camera feed visible
- [ ] Joint states publishing
- [ ] TF tree complete
- [ ] Policy outputs reasonable actions
- [ ] Adapter commands arm safely
- [ ] Base responds to commands
- [ ] Gripper opens/closes
- [ ] Emergency stop works

## Support

For issues:
- Check GitHub Issues: [repository-url]/issues
- Review logs in `~/.ros/log/`
- Ask on ROS Answers with tag `mobile-manipulation`

---

*Happy manipulating!* 🤖
