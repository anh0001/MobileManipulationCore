# AGENTS.md - AI Assistant Guide

This file describes how to work effectively in the MobileManipulationCore repo. Keep changes focused, follow ROS 2 conventions, and avoid touching third-party code unless explicitly requested.

## Project Summary

MobileManipulationCore is a ROS 2 Humble workspace that integrates perception, VLA policy inference, and action adaptation for mobile manipulation robots. It targets Ubuntu 22.04 and Jetson AGX Orin, with optional remote policy inference.

## Repo Map

- `src/manipulation_perception/`: C++ perception node
- `src/manipulation_policy/`: Python policy inference
- `src/manipulation_adapter/`: C++ action adapter
- `src/manipulation_msgs/`: Custom messages/actions
- `src/manipulation_bringup/`: Launch files
- `config/`: YAML configuration
- `docs/`: Design/usage/API docs
- `sim/`: Simulation assets
- `deployment/`: Docker/K8s configs
- `tests/`: Unit/integration tests
- `src/third_party/lerobot/`: Vendored dependency (avoid edits)

## Guardrails

- Do not modify `src/third_party/lerobot/` unless the user explicitly asks.
- Prefer small, reviewable changes; explain risky or wide-reaching edits.
- Keep ROS 2 package structure intact: update `CMakeLists.txt`, `package.xml`, or `setup.py` when adding sources.

## Build and Test

```bash
# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Test
colcon test
colcon test-result --verbose
```

## Common Tasks

- Add a new node:
  - C++: place under the package `src/` and update `CMakeLists.txt`.
  - Python: add module under the package and update `setup.py`/entry points.
- Add messages/actions:
  - Define under `src/manipulation_msgs/msg/` or `src/manipulation_msgs/action/`.
  - Update `src/manipulation_msgs/CMakeLists.txt` and rebuild.
- Update launch/config:
  - Launch files live in `src/manipulation_bringup/launch/`.
  - Configs live in `config/` and are referenced by launch files.

## Style

- C++: ROS 2 style guide; use `ament_clang_format` where applicable.
- Python: PEP 8; format with `black`, lint with `flake8`.
- Commits: Conventional Commits (`feat:`, `fix:`, `docs:`, etc.).

## References

- `README.md` for overview and usage
- `docs/design.md` for architecture details
- `docs/usage.md` for run/config instructions
- `docs/api_reference.md` for ROS 2 interfaces

