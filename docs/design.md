# Mobile Manipulator Intelligence Stack Design

## Overview

This document describes the architecture, design decisions, and implementation details for the MobileManipulationCore repository.

## Table of Contents

1. [Repository Strategy](#repository-strategy)
2. [Architecture](#architecture)
3. [ROS 2 Integration](#ros-2-integration)
4. [Deployment Profiles](#deployment-profiles)
5. [Engineering Practices](#engineering-practices)

## Repository Strategy

### Monorepo Approach

We use a **monorepo** structure where all manipulation modules (perception, policy, adapter, etc.) live in a single Git repository as separate ROS 2 packages.

**Benefits:**
- Easier coordination for small teams (1-3 developers)
- Centralized versioning and CI/CD
- Guaranteed compatibility between components
- Simplified dependency management

**Trade-offs:**
- All modules share the same release cycle
- Cannot independently version components

### External Dependencies

The existing `ranger-garden-assistant` base repository is treated as an external dependency. We interface only through ROS 2 messages/actions, with no code-level integration.

**Interface Boundary:** ROS 2 API surface (topics, services, actions)

This clean separation means:
- Base navigation + Piper driver stack runs independently
- Our manipulation stack provides the "brain" for manipulation
- Updates to one don't require modifying the other (as long as ROS interfaces remain consistent)

## Architecture

### Data Flow Pipeline

```
Sensors/State → Perception → Policy → Adapter → Hardware Controllers
     ↓              ↓           ↓         ↓
  (Raw data)  (Observation) (Action) (Commands)
```

#### 1. Sensors & State (External Base)

**Inputs from base stack:**
- Camera images (RGB, depth)
- Joint states
- TF transforms
- Odometry
- Optional: LiDAR, other sensors

These are the "eyes and proprioception" of the system.

#### 2. Perception Module

**Package:** `manipulation_perception`

**Responsibilities:**
- Subscribe to raw sensor data
- Preprocess images (resize, normalize)
- Aggregate multi-sensor information
- Generate structured observations for policy
- Optional: Object detection, state estimation

**Output:** Observation message containing all necessary info for decision-making

#### 3. Policy Module (High-Level Intelligence)

**Package:** `manipulation_policy`

**Responsibilities:**
- Run VLA model inference (OpenVLA, LeRobot)
- Map vision + state → robot actions
- Support both local and remote inference

**Key Features:**
- Leverages pre-trained models from HuggingFace
- Autoregressive action generation
- Language-conditioned capabilities (optional)

**Output:** High-level action (e.g., target end-effector pose, joint deltas, gripper command)

#### 4. Adapter Module (Critical Bridge)

**Package:** `manipulation_adapter`

**Responsibilities:**
- Map policy output to full mobile-manipulator action space
- Expand arm-only actions to base + arm + gripper commands
- TF-aware coordinate transformations
- Safety checks and constraint enforcement
- Inverse kinematics / motion planning

**Safety Features:**
- Joint limit checking
- Collision avoidance
- Singularity detection
- Workspace boundaries
- Timeout/fallback handling

**Output:** Low-level commands:
- `geometry_msgs/Twist` for base
- `NavigateToPose` action for navigation
- Joint trajectory for arm
- Gripper commands

### Observation Schema

Conceptual structure of what the policy needs:

```yaml
Observation:
  - Images: RGB (required), depth (optional)
  - Proprioceptive: joint angles, gripper state
  - Base state: pose, velocity (optional)
  - Task context: language instruction (optional)
  - Environment: costmap info (optional)
```

Implementation: Not a single monolithic message; components subscribe to individual topics and assemble observations internally.

### Action Schema

Policy output structure:

```yaml
PolicyOutput:
  - EEF target: pose or velocity (geometry_msgs/Pose)
  - Joint deltas: alternative to pose-based (float64[])
  - Gripper command: open/close/continuous (float64)
  - Base hint: optional velocity suggestion (geometry_msgs/Twist)
  - Metadata: reference frame, confidence
```

See `manipulation_msgs/msg/PolicyOutput.msg` for implementation.

### Adapter Logic

The adapter implements:

1. **Reachability Check:** Is EEF target within arm workspace?
2. **Base Coordination:** If out of reach → command base to reposition
3. **IK Solving:** Convert EEF pose → joint angles
4. **Trajectory Generation:** Smooth motion respecting velocity/acceleration limits
5. **Safety Filtering:** Reject unsafe commands

**TF-Aware:**
- Transforms between camera frame, base_link, map
- Uses current robot state from TF tree
- Ensures consistent coordinate interpretation

## ROS 2 Integration

### Topic Interface

**Subscribed Topics (from base):**
- `/camera/color/image_raw` (sensor_msgs/Image)
- `/camera/depth/image_raw` (sensor_msgs/Image)
- `/joint_states` (sensor_msgs/JointState)
- `/tf`, `/tf_static` (TF transforms)
- `/odom` (nav_msgs/Odometry)

**Published Topics (to hardware):**
- `/cmd_vel` (geometry_msgs/Twist) - base velocity
- `/manipulation/policy_output` (manipulation_msgs/PolicyOutput) - internal

**Action Clients:**
- `/navigate_to_pose` (nav2_msgs/action/NavigateToPose)
- `/arm_controller/follow_joint_trajectory` (control_msgs/action/FollowJointTrajectory)
- `/move_action` (moveit_msgs/action/MoveGroup)

### TF Frame Expectations

Required frames:
- `map` - global frame
- `odom` - odometry frame
- `base_link` - robot base
- `piper_link_*` - arm links
- `piper_gripper_base` - end-effector
- `camera_link` - camera mount
- `camera_color_optical_frame` - camera optical frame

## Deployment Profiles

### Profile A: Jetson-Only (On-Device)

**Configuration:**
- All components run on Jetson AGX Orin
- Single ROS 2 domain
- Local model inference

**Pros:**
- No network dependency
- Lower latency
- Simpler deployment

**Cons:**
- Limited compute for large models
- May require model optimization (TensorRT, quantization)
- Possibly reduced inference frequency (5-10 Hz)

**Launch:**
```bash
ros2 launch manipulation_bringup core_launch.py
```

### Profile B: Split (Jetson + Remote Server)

**Configuration:**
- Jetson: perception, adapter, low-level control
- Remote GPU server: policy inference
- Communication via ROS 2 over network or gRPC

**Pros:**
- Leverage powerful GPU for larger/faster models
- Multiple robots can share inference server
- Better for development/testing

**Cons:**
- Network latency (typ. 50-100ms)
- Requires robust connectivity
- More complex deployment

**Mitigation:**
- Image compression
- Latency compensation in adapter
- Timeout/fallback mechanisms
- Secure communication (SROS2, VPN)

**Launch:**
```bash
# On GPU server:
ros2 launch manipulation_policy remote_server_launch.py

# On Jetson:
ros2 launch manipulation_bringup core_launch.py use_remote_policy:=true
```

## Engineering Practices

### Continuous Integration

**GitHub Actions CI:**
- Build verification (colcon build)
- Linters (C++: ament_cpplint, Python: flake8)
- Static analysis (clang-tidy, pylint)
- Unit tests (GTest, pytest)
- Integration tests (launch testing)

**Quality Gates:**
- All tests must pass
- No lint errors
- Code coverage targets (optional)

### Dependency Management

**ROS Dependencies:**
- Managed via `package.xml` and `rosdep`
- Standard ROS 2 packages from apt

**Python ML Dependencies:**
- `PYTHONUSERBASE`-scoped installs with `requirements.txt` to avoid ROS 2 system conflicts
- Pin exact versions for reproducibility
- Dedicated user base for the policy server (e.g., `~/.local/ros2_humble`)

**Piper Dependencies:**
- `config/piper_versions.yaml` pins specific commits
- `scripts/install_piper.sh` fetches and builds

### Testing Strategy

1. **Unit Tests:** Test individual functions (IK, transforms, safety checks)
2. **Integration Tests:** Test node communication, launch files
3. **Simulation Tests:** End-to-end manipulation in Gazebo
4. **Hardware Tests:** Final validation on real robot

### Safety Considerations

- **Timeout Handling:** Stop robot if policy output not received
- **Fallback Modes:** Safe default behaviors
- **Constraint Checking:** Joint limits, workspace bounds
- **Emergency Stop:** Ability to halt all motion
- **Logging:** Comprehensive logs for debugging

## Future Extensions

### RoboNeuron Integration

Our architecture aligns with RoboNeuron's modular approach:
- Policy node = "reasoning" module (VLA)
- Adapter = "control" module
- Can integrate LLM orchestrator via services

### Additional Sensors

- Tactile feedback
- Force/torque sensing
- Additional cameras

### Advanced Features

- Multi-task learning
- Dynamic obstacle avoidance
- Human-robot collaboration
- Fleet coordination

## References

- [OpenVLA](https://openvla.github.io/)
- [LeRobot](https://huggingface.co/lerobot)
- [RoboNeuron](https://arxiv.org/abs/2512.10394)
- [Piper ROS](https://github.com/agilexrobotics/piper_ros)

---

*For implementation details, see source code and inline documentation.*
