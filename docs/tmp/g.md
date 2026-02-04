# Copilot instructions for MobileManipulationCore

## Big picture architecture
- ROS 2 monorepo with a perception→policy→adapter pipeline. Core nodes live in src/:
  - manipulation_perception/src/perception_node.cpp subscribes to camera/depth/joint state topics and prepares observations.
  - manipulation_policy/manipulation_policy/policy_node.py publishes manipulation_msgs/PolicyOutput.msg to /manipulation/policy_output (currently a stub model).
  - manipulation_adapter/src/adapter_node.cpp consumes PolicyOutput and publishes /cmd_vel plus (TODO) arm/gripper actions, with a safety timeout.
- External base stack (navigation + hardware drivers) is a strict boundary; integration is via ROS 2 topics/actions only (see docs/design.md).
- Dataflow and action/observation schemas are defined in docs/design.md and manipulation_msgs/msg/PolicyOutput.msg.

## Key developer workflows
- Build ROS 2 workspace: rosdep install --from-paths src --ignore-src -r -y then colcon build --symlink-install (README.md).
- Run stack: ros2 launch manipulation_bringup core_launch.py (real robot) or sim_launch.py (sim). Launch args in src/manipulation_bringup/launch/core_launch.py.
- Tests: colcon test and colcon test-result --verbose (tests/README.md). Integration tests live in tests/integration/.
- Deployment options (Jetson vs split policy server) are documented in deployment/README.md and docs/usage.md.

## Project-specific conventions and patterns
- Parameters are declared and pulled at node startup; launch files override them (core_launch.py sets camera topic, remote policy, base/ee frames, safety timeouts).
- Policy output always goes through manipulation_msgs/PolicyOutput.msg; adapter checks flags like has_eef_target/has_base_hint before acting.
- Safety behavior: adapter publishes zero cmd_vel if no policy output for safety_timeout_sec (adapter_node.cpp).
- Topic names are configurable via parameters but defaults match docs/design.md (e.g., /camera/color/image_raw, /joint_states).

## Integration points & dependencies
- Third-party VLA stack is vendored as a git submodule at src/third_party/lerobot; avoid editing unless intentionally updating the submodule (src/third_party/README.md).
- Optional remote policy inference is enabled by launch arg use_remote_policy and parameter remote_url (core_launch.py, policy_node.py).
- Docker and compose configs are in deployment/docker/; K8s stubs in deployment/k8s/.

## Where to look first
- Architecture and interfaces: docs/design.md, docs/api_reference.md
- Config: config/robot_params.yaml, config/policy_params.yaml
- Launch: src/manipulation_bringup/launch/core_launch.py
