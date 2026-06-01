# MobileManipulationCore

MobileManipulationCore is the core "brain" for mobile manipulation, providing high-level AI/control capabilities for robots combining mobility and manipulation. This repository contains a ROS 2-based system that integrates vision-language-action models (VLA) with mobile manipulator hardware.

## Overview

This stack enables intelligent mobile manipulation by combining:
- **Perception**: Sensor processing and observation generation for policy models
- **Policy**: ML-based decision making using models like OpenVLA/LeRobot
- **Visual Servo**: Feature-based closed-loop control path for target centering/alignment
- **Adapter**: Safe coordination of arm and base movements with TF-aware planning
- **Integration**: Clean ROS 2 interfaces for hardware-agnostic deployment

## Key Features

- 🤖 **Generalist Policy Integration**: Built-in support for OpenVLA and HuggingFace LeRobot models
- 🔧 **Hardware Agnostic**: Standard ROS 2 interfaces work with any compatible mobile manipulator
- 🎯 **TF-Aware Adaptation**: Intelligent coordination of base and arm using transform trees
- 🎮 **Dual Control Modes**: Switch between `vla` and `visual_servo` in `core_launch.py`
- 🚀 **Flexible Deployment**: Run fully on-device (Jetson) or with remote GPU inference
- 🔒 **Production Ready**: Testing, Docker deployment, and security features
- 📦 **Monorepo Design**: All packages in one place for easy development

## Architecture

```
                                  control_mode:=vla
Prompt Input ───────┐
                    v
Sensors/TF ──→ Perception ──→ Observation ──→ Policy (VLA) ──┐
                                                              │
                        control_mode:=visual_servo            v
RGB + CameraInfo ──→ Detection Client (manipulation_detection) ──→ /manipulation/target_detections
Target Prompt ───────────────────────────────────────────────────→ /visual_servo/target_prompt
                                                                        │
                                                                        v
                                                    Visual Servo Node ─→ PolicyOutput
                                                                        │
                                                                        v
                                                                     Adapter
                                                                        v
                                                             Base + Arm Controllers
```

The system follows a modular pipeline:
1. **Sensors & State**: Camera, depth, joint states, TF from base stack
2. **Perception**: Processes raw data into structured `Observation` for VLA mode
3. **Control Generation (runtime switch via `control_mode`)**:
   - `vla`: task prompt + observation -> policy inference
   - `visual_servo`: `manipulation_detection` builds `Detection2DArray` from RGB/prompt and feeds visual-servo control
4. **Adapter**: Consumes unified `PolicyOutput` and maps to feasible base+arm commands with safety checks
5. **Execution**: Commands sent via standard ROS 2 topics/actions

## Quick Start

### Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10+
- NVIDIA Jetson AGX Orin (for on-device deployment) or GPU-enabled machine

### Robot Hardware Stack (required for real-robot runs)

MobileManipulationCore is the high-level "brain" only — it does **not** include
the robot bringup (drivers, controllers, MoveIt, sensors). For any real-robot
run it requires the companion hardware stack:

- **Repo:** [anh0001/ranger-garden-assistant](https://github.com/anh0001/ranger-garden-assistant.git)
  (AgileX Ranger Mini 3.0 base + PiPER 6-DOF arm, Livox Mid-360, RealSense D405).
- **Location:** clone it as a **sibling folder** next to this repo, e.g.:

  ```
  ~/codes/
  ├── MobileManipulationCore        # this repo
  └── ranger-garden-assistant       # robot stack (sibling)
  ```

  ```bash
  cd ~/codes
  git clone https://github.com/anh0001/ranger-garden-assistant.git
  ```

- **Bring up the real robot first**, then launch this stack on top. From the
  `ranger-garden-assistant` workspace root:

  ```bash
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch robofi_bringup ranger_complete_bringup.launch.py
  ```

  This provides the drivers, `move_group`, `servo_node`, arm/gripper
  controllers, and camera/LiDAR that MobileManipulationCore commands. The
  Visual Servo adapter publishes to `/servo_node/delta_twist_cmds` exposed by
  this bringup. See the [robot stack README](https://github.com/anh0001/ranger-garden-assistant)
  for build and hardware setup.

### Installation

```bash
# Clone the repository
git clone <repository-url>
cd MobileManipulationCore

# Initialize and update submodules recursively
git submodule update --init --recursive

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Install Piper arm dependencies, if using Piper (optional)
./scripts/install_piper.sh

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### Running the Stack

**Simulation Mode:**
```bash
ros2 launch manipulation_bringup sim_launch.py
```

> **Prerequisite:** the [robot hardware stack](#robot-hardware-stack-required-for-real-robot-runs)
> (`ranger-garden-assistant`) must already be running its complete bringup
> (`ros2 launch robofi_bringup ranger_complete_bringup.launch.py`) before
> starting any real-robot mode below.

**Real Robot (VLA Mode, default):**
```bash
ros2 launch manipulation_bringup core_launch.py
```

**Real Robot (Visual Servo Mode):**
```bash
ros2 launch manipulation_bringup core_launch.py control_mode:=visual_servo
```

**Remote Grounding DINO Server (for Visual Servo detections):**
```bash
# On remote GPU machine
python3 -m manipulation_detection.detection_server --host 0.0.0.0 --port 30543
```

> **Note:** In visual-servo mode, `remote_detection_client` sends JPEG-compressed frames to the remote detector and republishes detections on `/manipulation/target_detections` (`vision_msgs/msg/Detection2DArray`). Runtime detection prompt topic is `/visual_servo/target_prompt`.

**Visual Servo Node Only (Debug/Bench):**
```bash
ros2 launch manipulation_visual_servo visual_servo_debug.launch.py
```

**Split Deployment (Jetson + Remote Server):**

> **Note:** For detailed OpenVLA remote server setup instructions, see [docs/openvla_remote.md](docs/openvla_remote.md)

```bash
# On remote GPU server:
ros2 launch manipulation_policy remote_server_launch.py
# Or, if ROS 2 CLI is not available:
# python3 -m manipulation_policy.policy_server

# On robot:
ros2 launch manipulation_bringup core_launch.py use_remote_policy:=true
```

**Interactive Task Prompts (OpenVLA via policy node):**
```bash
# In another terminal (after launching core stack):
ros2 run manipulation_policy task_prompt_cli

# Example prompts:
# > pick up the bottle
# > place the bottle on the table
# > stop
```

**OpenVLA Frame Note (Bridge-Orig):**
- `policy_node` uses `arm_base_frame` as `reference_frame` (default: `piper_base_link`), not `camera_link`.
- `core_launch.py` passes this from `config/robot_params.yaml` -> `robot.frames.arm_base`.
- Keep `adapter_node.arm_base_frame` aligned with the same frame.
- Arm execution is runtime-selectable via `arm_execution_mode` (`moveit_servo` default, `move_group` fallback).
- Optional Bridge-V2 virtual frames can be enabled in `config/robot_params.yaml` (`bridge_v2_virtual_frames.enabled`).
  This publishes `bridge_v2_camera_virtual` attached to `piper_camera_link` and a derived
  `bridge_v2_base_virtual`; policy outputs then use `bridge_v2_base_virtual` as `reference_frame`.

## Repository Structure

```
MobileManipulationCore/
├── src/                          # ROS 2 packages
│   ├── manipulation_perception/  # Sensor processing
│   ├── manipulation_policy/      # ML policy inference
│   ├── manipulation_detection/   # Remote detection bridge/server for visual servo
│   ├── manipulation_adapter/     # Action-to-command mapping
│   ├── manipulation_visual_servo/  # Visual servo control node
│   ├── manipulation_msgs/        # Custom message definitions
│   └── manipulation_bringup/     # Launch files
├── scripts/                      # Installation and setup scripts
├── config/                       # Configuration files
├── docs/                         # Documentation
├── sim/                          # Simulation assets
├── deployment/                   # Docker and deployment configs
└── tests/                        # Unit and integration tests
```

## Documentation

- [Design Document](docs/design.md) - Detailed architecture and design decisions
- [Usage Guide](docs/usage.md) - How to run, configure, and extend the stack
- [API Reference](docs/api_reference.md) - ROS interfaces (topics/services/actions)
- [Grounding DINO Remote Guide](docs/grounding_dino_remote.md) - Remote detector setup, contracts, and tuning

## ROS 2 Interface

### Key Topics

**Inputs (from base stack):**
- `/camera/image_raw` - RGB camera feed
- `/joint_states` - Robot joint states
- `/tf`, `/tf_static` - Transform tree

**Outputs (to hardware controllers):**
- `/cmd_vel` - Base velocity commands
- `/navigate_to_pose` - Navigation goals (Nav2 action)
- Arm control via MoveIt Servo (`/servo_node/delta_twist_cmds`) or `FollowJointTrajectory`/MoveGroup fallback

See [API Reference](docs/api_reference.md) for complete interface documentation.

## Deployment Options

### Jetson-Only (On-Device)
All components run on the robot's Jetson AGX Orin. Best for:
- Scenarios requiring no network connectivity
- Latency-critical applications
- Simplified deployment

### Split Deployment (Edge + Cloud)
Policy inference runs on remote GPU server, other components on Jetson. Best for:
- Larger/more capable models
- Multiple robots sharing inference
- Development and testing

## Development

### Running Tests

```bash
# Build with tests
colcon build --symlink-install

# Run all tests
colcon test

# View test results
colcon test-result --verbose
```

### Visual Servo Smoke Test

```bash
# Build relevant packages
colcon build --symlink-install --packages-up-to manipulation_visual_servo manipulation_bringup

# Run package tests (lint + package-level tests)
colcon test --packages-select manipulation_visual_servo manipulation_bringup
colcon test-result --verbose

# Start remote detector server (on remote GPU machine)
python3 -m manipulation_detection.detection_server --host 0.0.0.0 --port 30543

# Launch full stack in visual servo mode
ros2 launch manipulation_bringup core_launch.py control_mode:=visual_servo

# In another terminal, verify visual-servo outputs
ros2 run manipulation_detection detection_prompt_cli
ros2 topic hz /manipulation/target_detections
ros2 topic echo /visual_servo/state
ros2 topic hz /manipulation/policy_output
```

## License

Apache License 2.0. See `LICENSE`.

## Citation

If you use this work in your research, please cite:

```bibtex
@software{mobilemanipulationcore,
  title={MobileManipulationCore: A Modular Intelligence Stack for Mobile Manipulation},
  author={Anhar Risnumawan},
  year={2026},
  url={[repository-url]}
}
```

## Acknowledgments

- Built on [OpenVLA](https://openvla.github.io/) and [LeRobot](https://huggingface.co/lerobot)
- Compatible with [Piper](https://github.com/agilexrobotics/piper_ros) robotic arm
- Follows [RoboNeuron](https://arxiv.org/abs/2512.10394) principles for modularity

---

**Status**: 🚧 Under active development

Built with ❤️ for the robotics community
