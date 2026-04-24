# CLAUDE.md - AI Assistant Guide

This document provides context for AI assistants working with the MobileManipulationCore codebase.

## Project Overview

MobileManipulationCore is a ROS 2-based intelligent control system for mobile manipulation robots. It integrates vision-language-action (VLA) models with hardware controllers to enable autonomous mobile manipulation tasks.

**Key Technologies:**
- ROS 2 Humble
- Python 3.10+ and C++17
- OpenVLA / HuggingFace LeRobot for policy inference
- NVIDIA Jetson AGX Orin for edge deployment
- PyTorch for ML models

## Architecture

The system follows a modular pipeline with four main components:

```
Sensors → Perception → Policy (VLA) → Adapter → Controllers
```

1. **Perception** ([src/manipulation_perception](src/manipulation_perception)): Processes raw sensor data (camera, depth, joint states, TF) into structured observations for the policy
2. **Policy** ([src/manipulation_policy](src/manipulation_policy)): Runs VLA models (OpenVLA/LeRobot) to generate high-level actions
3. **Adapter** ([src/manipulation_adapter](src/manipulation_adapter)): Maps policy actions to safe, feasible commands for base and arm with TF-aware planning
4. **Messages** ([src/manipulation_msgs](src/manipulation_msgs)): Custom ROS 2 message and action definitions
5. **Bringup** ([src/manipulation_bringup](src/manipulation_bringup)): Launch files for various deployment configurations

## Directory Structure

```
MobileManipulationCore/
├── src/
│   ├── manipulation_perception/   # C++ perception node
│   ├── manipulation_policy/       # Python policy inference
│   ├── manipulation_adapter/      # C++ action adapter
│   ├── manipulation_msgs/         # ROS 2 message definitions
│   ├── manipulation_bringup/      # Launch configurations
│   └── third_party/lerobot/       # Git submodule for LeRobot
├── scripts/                       # Setup and installation scripts
├── config/                        # YAML configuration files
├── docs/                          # Detailed documentation
├── sim/                          # Gazebo simulation assets
├── deployment/                   # Docker/K8s deployment configs
└── tests/                        # Unit and integration tests
```

## Key Files to Know

### Launch Files
- [src/manipulation_bringup/launch/core_launch.py](src/manipulation_bringup/launch/core_launch.py) - Main robot launch
- [src/manipulation_bringup/launch/sim_launch.py](src/manipulation_bringup/launch/sim_launch.py) - Simulation launch

### Configuration Files
- [config/robot_params.yaml](config/robot_params.yaml) - Robot hardware parameters
- [config/policy_params.yaml](config/policy_params.yaml) - Policy model configuration
- [config/piper_versions.yaml](config/piper_versions.yaml) - Piper arm version mappings

### Core Nodes
- [src/manipulation_perception/src/perception_node.cpp](src/manipulation_perception/src/perception_node.cpp) - Perception processing
- [src/manipulation_policy/manipulation_policy/policy_node.py](src/manipulation_policy/manipulation_policy/policy_node.py) - Policy inference
- [src/manipulation_adapter/src/adapter_node.cpp](src/manipulation_adapter/src/adapter_node.cpp) - Action adaptation

### Documentation
- [README.md](README.md) - Main project documentation
- [docs/design.md](docs/design.md) - Architecture and design decisions
- [docs/usage.md](docs/usage.md) - Usage instructions
- [docs/api_reference.md](docs/api_reference.md) - ROS 2 interface documentation

## ROS 2 Interface

### Published Topics (System Outputs)
- `/cmd_vel` (geometry_msgs/Twist) - Base velocity commands
- `/manipulator/joint_trajectory` (trajectory_msgs/JointTrajectory) - Arm trajectory commands
- `/policy/output` (manipulation_msgs/PolicyOutput) - Raw policy outputs

### Subscribed Topics (System Inputs)
- `/camera/color/image_raw` (sensor_msgs/Image) - RGB camera feed
- `/camera/depth/image_rect_raw` (sensor_msgs/Image) - Depth images
- `/joint_states` (sensor_msgs/JointState) - Robot joint states
- `/tf`, `/tf_static` (tf2_msgs/TFMessage) - Transform tree

### Actions
- `/navigate_to_pose` (nav2_msgs/NavigateToPose) - Base navigation
- `/manipulator/follow_joint_trajectory` (control_msgs/FollowJointTrajectory) - Arm control
- `/execute_manipulation_task` (manipulation_msgs/ManipulationTask) - High-level task execution

## Development Workflow

### Building the Workspace

```bash
# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build all packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### Running Tests

```bash
# Build with tests
colcon build --symlink-install

# Run all tests
colcon test

# View results
colcon test-result --verbose
```

### Code Style
- **C++**: ROS 2 style guide, use `ament_clang_format`
- **Python**: PEP 8, use `flake8` and `black`
- **Commits**: Conventional Commits format (feat:, fix:, docs:, etc.)

## Common Tasks

### Adding a New ROS 2 Node

1. Choose the appropriate package based on functionality
2. For C++ nodes: Add to `src/` and update `CMakeLists.txt`
3. For Python nodes: Add to package directory and update `setup.py`
4. Update launch files in `manipulation_bringup` as needed

### Adding Custom Messages

1. Define in [src/manipulation_msgs/msg/](src/manipulation_msgs/msg/) or [src/manipulation_msgs/action/](src/manipulation_msgs/action/)
2. Update [src/manipulation_msgs/CMakeLists.txt](src/manipulation_msgs/CMakeLists.txt) to include new messages
3. Rebuild the workspace
4. Import in Python: `from manipulation_msgs.msg import YourMessage`
5. Include in C++: `#include <manipulation_msgs/msg/your_message.hpp>`

### Modifying Policy Integration

- Policy configuration: [config/policy_params.yaml](config/policy_params.yaml)
- Policy node implementation: [src/manipulation_policy/manipulation_policy/policy_node.py](src/manipulation_policy/manipulation_policy/policy_node.py)
- LeRobot submodule: [src/third_party/lerobot/](src/third_party/lerobot/)

### Working with Simulation

- Launch simulation: `ros2 launch manipulation_bringup sim_launch.py`
- Gazebo worlds: [sim/worlds/](sim/worlds/)
- Simulation launch file: [sim/launch/gazebo_launch.py](sim/launch/gazebo_launch.py)

## Deployment Options

### On-Device (Jetson Only)
All processing on robot's Jetson AGX Orin. Best for latency-critical or offline scenarios.

```bash
ros2 launch manipulation_bringup core_launch.py
```

### Split Deployment (Edge + Cloud)
Policy inference on remote GPU server, other components on Jetson.

```bash
# On remote server:
ros2 launch manipulation_policy remote_server_launch.py

# On robot:
ros2 launch manipulation_bringup core_launch.py use_remote_policy:=true
```

### Docker Deployment
- Jetson container: [deployment/docker/Dockerfile.jetson](deployment/docker/Dockerfile.jetson)
- Policy server container: [deployment/docker/Dockerfile.policy](deployment/docker/Dockerfile.policy)
- Docker Compose: [deployment/docker/docker-compose.yml](deployment/docker/docker-compose.yml)

## Important Considerations

### Hardware Dependencies
- Piper robotic arm support via [scripts/install_piper.sh](scripts/install_piper.sh)
- NVIDIA GPU required for policy inference (Jetson or remote server)
- Intel RealSense or similar depth camera

### Performance
- Policy inference is compute-intensive (expect 10-20 Hz on Jetson AGX Orin)
- Perception node should run at camera framerate (30 Hz)
- Adapter node provides real-time safety checks (<10ms latency)

### Safety
- Adapter node enforces velocity limits and collision avoidance
- TF-aware planning prevents self-collisions
- Emergency stop functionality via ROS 2 services

### Testing
- Unit tests in [tests/unit/](tests/unit/)
- Integration tests in [tests/integration/](tests/integration/)
- CI/CD via GitHub Actions (builds, linting, testing)

## Debugging Tips

### Visualization
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Monitor topics
ros2 topic echo /policy/output
ros2 topic hz /camera/color/image_raw

# Check node status
ros2 node list
ros2 node info /perception_node
```

### Logging
- ROS 2 logging: `ros2 run rqt_console rqt_console`
- Log levels configurable in launch files
- Check system logs: `journalctl -u robot_service`

### Performance Profiling
```bash
# CPU/Memory usage
ros2 run ros2_monitor resource_monitor

# Latency analysis
ros2 topic delay /camera/color/image_raw
```

## External Resources

- [OpenVLA Documentation](https://openvla.github.io/)
- [LeRobot GitHub](https://github.com/huggingface/lerobot)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Piper Arm ROS Package](https://github.com/agilexrobotics/piper_ros)
- [RoboNeuron Paper](https://arxiv.org/abs/2512.10394) - Architectural inspiration

## Contact & Support

- GitHub Issues: Use for bug reports and feature requests
- Documentation: Check [docs/](docs/) directory for detailed guides
- Code Comments: Many files contain inline documentation

---

**Last Updated:** 2026-02-04
**ROS 2 Distribution:** Humble
**Target Platform:** Ubuntu 22.04 + Jetson AGX Orin
