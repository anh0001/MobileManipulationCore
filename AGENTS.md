# AGENTS.md - AI Assistant Guide

This file describes how to work effectively in the MobileManipulationCore repo. Keep changes focused, follow ROS 2 conventions, and avoid touching third-party code unless explicitly requested.

## Project Summary

MobileManipulationCore is a ROS 2 Humble workspace that integrates perception, VLA policy inference, and action adaptation for mobile manipulation robots. It targets Ubuntu 22.04 and Jetson AGX Orin, with optional remote policy inference.

## Repo Map

- `src/manipulation_perception/`: C++ perception node
- `src/manipulation_policy/`: Python policy inference
- `src/manipulation_adapter/`: C++ action adapter
- `src/manipulation_detection/`: Python detection service/client package
- `src/manipulation_visual_servo/`: C++ visual servoing package
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
- Use `rg`/`rg --files` for local discovery before broader scans.
- Prefer official docs or Context7 for API/framework behavior before adding new patterns.
- Never hardcode secrets, tokens, credentials, or deployment keys; use environment variables or existing secret mechanisms.
- Treat robot safety as a first-class review concern: check velocity limits, TF frames, parameter defaults, action timeouts, and failure behavior.

## Build and Test

```bash
# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Build one package after scoped changes
colcon build --symlink-install --packages-select <package>

# Test
colcon test
colcon test-result --verbose

# Test one package after scoped changes
colcon test --packages-select <package>
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
- Tests: use tests-first for behavior changes and bug fixes; keep coverage focused on changed behavior, edge cases, and failure modes.
- Reviews: lead with severity-ranked correctness, safety, regression, and missing-test findings before summaries.
- Commits: Conventional Commits (`feat:`, `fix:`, `docs:`, etc.).

## Codex Setup

- Project-local Codex configuration lives in `.codex/config.toml`.
- Custom Codex agents live in `.codex/agents/` and should stay read-only unless a future task explicitly calls for worker agents.
- This repo intentionally uses a lean Codex setup from Everything Claude Code practices; do not copy full ECC hooks, global sync scripts, generic prompt packs, or Claude-specific assets into this repo.

## References

- `README.md` for overview and usage
- `docs/design.md` for architecture details
- `docs/usage.md` for run/config instructions
- `docs/api_reference.md` for ROS 2 interfaces
