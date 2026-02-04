# MobileManipulationCore

MobileManipulationCore is the core "brain" for mobile manipulation, providing high-level AI/control capabilities for robots combining mobility and manipulation. This repository contains a ROS 2-based system that integrates vision-language-action models (VLA) with mobile manipulator hardware.

## Overview

This stack enables intelligent mobile manipulation by combining:
- **Perception**: Sensor processing and observation generation for policy models
- **Policy**: ML-based decision making using models like OpenVLA/LeRobot
- **Adapter**: Safe coordination of arm and base movements with TF-aware planning
- **Integration**: Clean ROS 2 interfaces for hardware-agnostic deployment

## Key Features

- 🤖 **Generalist Policy Integration**: Built-in support for OpenVLA and HuggingFace LeRobot models
- 🔧 **Hardware Agnostic**: Standard ROS 2 interfaces work with any compatible mobile manipulator
- 🎯 **TF-Aware Adaptation**: Intelligent coordination of base and arm using transform trees
- 🚀 **Flexible Deployment**: Run fully on-device (Jetson) or with remote GPU inference
- 🔒 **Production Ready**: Testing, Docker deployment, and security features
- 📦 **Monorepo Design**: All packages in one place for easy development

## Architecture

```
Sensors → Perception → Policy (VLA) → Adapter → Base + Arm Controllers
            ↓            ↓              ↓
         Observation  Action     Low-level Commands
```

The system follows a modular pipeline:
1. **Sensors & State**: Camera, depth, joint states, TF from base stack
2. **Perception**: Processes raw data into structured observations
3. **Policy**: Generates high-level actions using VLA models
4. **Adapter**: Maps actions to feasible base+arm commands with safety checks
5. **Execution**: Commands sent via standard ROS 2 topics/actions

## Quick Start

### Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10+
- NVIDIA Jetson AGX Orin (for on-device deployment) or GPU-enabled machine

### Installation

```bash
# Clone the repository
git clone <repository-url>
cd MobileManipulationCore

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Install Piper arm dependencies (if using Piper)
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

**Real Robot (Jetson-only):**
```bash
ros2 launch manipulation_bringup core_launch.py
```

**Split Deployment (Jetson + Remote Server):**
```bash
# On remote GPU server:
ros2 launch manipulation_policy remote_server_launch.py
# Or, if ROS 2 CLI is not available:
# python3 -m manipulation_policy.policy_server

# On robot:
ros2 launch manipulation_bringup core_launch.py use_remote_policy:=true
```

## Repository Structure

```
MobileManipulationCore/
├── src/                          # ROS 2 packages
│   ├── manipulation_perception/  # Sensor processing
│   ├── manipulation_policy/      # ML policy inference
│   ├── manipulation_adapter/     # Action-to-command mapping
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

## ROS 2 Interface

### Key Topics

**Inputs (from base stack):**
- `/camera/image_raw` - RGB camera feed
- `/joint_states` - Robot joint states
- `/tf`, `/tf_static` - Transform tree

**Outputs (to hardware controllers):**
- `/cmd_vel` - Base velocity commands
- `/navigate_to_pose` - Navigation goals (Nav2 action)
- Arm control via `FollowJointTrajectory` action

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
