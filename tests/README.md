# Tests

This directory contains test suites for the MobileManipulationCore stack.

## Directory Structure

```
tests/
├── unit/           # Unit tests for individual components
└── integration/    # End-to-end integration tests
```

## Running Tests

### All Tests

```bash
# Build with tests
cd ~/manipulation_ws
colcon build --symlink-install

# Run all tests
colcon test

# View results
colcon test-result --verbose
```

### Specific Package

```bash
# Test a single package
colcon test --packages-select manipulation_adapter

# With output
colcon test --packages-select manipulation_adapter --event-handlers console_direct+
```

### Coverage

```bash
# Install coverage tools
pip install coverage pytest-cov

# Run with coverage
colcon test --packages-select manipulation_policy --pytest-args --cov

# Generate coverage report
coverage report
coverage html  # Creates htmlcov/index.html
```

## Unit Tests

Unit tests verify individual functions and components in isolation.

### C++ Unit Tests (GTest)

Located within each package's `test/` directory.

**Example:** `src/manipulation_adapter/test/test_adapter.cpp`

```cpp
#include <gtest/gtest.h>
#include "manipulation_adapter/safety_checks.hpp"

TEST(SafetyTest, JointLimitCheck) {
  std::vector<double> joint_angles = {0.0, 0.5, 1.0};
  std::vector<double> lower_limits = {-1.0, -1.0, -1.0};
  std::vector<double> upper_limits = {1.0, 1.0, 1.0};

  EXPECT_TRUE(checkJointLimits(joint_angles, lower_limits, upper_limits));
}

TEST(SafetyTest, JointLimitViolation) {
  std::vector<double> joint_angles = {2.0, 0.5, 1.0};  // Exceeds limit
  std::vector<double> lower_limits = {-1.0, -1.0, -1.0};
  std::vector<double> upper_limits = {1.0, 1.0, 1.0};

  EXPECT_FALSE(checkJointLimits(joint_angles, lower_limits, upper_limits));
}
```

**Run:**
```bash
colcon test --packages-select manipulation_adapter
```

### Python Unit Tests (pytest)

Located within each Python package's `test/` directory.

**Example:** `src/manipulation_policy/test/test_policy.py`

```python
import pytest
import numpy as np
from manipulation_policy.policy_node import PolicyNode

def test_image_preprocessing():
    """Test image preprocessing pipeline."""
    # Create dummy image
    image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

    # Process
    processed = preprocess_image(image, target_size=(224, 224))

    # Verify
    assert processed.shape == (224, 224, 3)
    assert processed.dtype == np.float32
    assert processed.min() >= 0.0 and processed.max() <= 1.0

def test_policy_output_format():
    """Test policy output has correct format."""
    # Mock policy inference
    output = mock_policy_inference()

    # Verify output structure
    assert 'eef_target' in output
    assert 'gripper_command' in output
    assert 0.0 <= output['gripper_command'] <= 1.0
```

**Run:**
```bash
colcon test --packages-select manipulation_policy
```

## Integration Tests

Integration tests verify multiple components working together.

### Launch Testing

Test that launch files start correctly and nodes communicate.

**Example:** `tests/integration/test_core_launch.py`

```python
import unittest
import pytest
import rclpy
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
import launch_testing

@pytest.mark.launch_test
def generate_test_description():
    # Start nodes
    perception_node = Node(
        package='manipulation_perception',
        executable='perception_node',
    )

    return LaunchDescription([
        perception_node,
        ReadyToTest(),
    ])

class TestCoreStack(unittest.TestCase):
    def test_nodes_start(self):
        """Test that all nodes start successfully."""
        # Nodes started by launch system
        pass

    def test_topics_published(self, proc_output):
        """Test that expected topics are being published."""
        rclpy.init()
        node = rclpy.create_node('test_node')

        # Wait for topic
        topic_found = node.wait_for_message(
            '/manipulation/policy_output',
            timeout_sec=5.0
        )

        self.assertTrue(topic_found)
        node.destroy_node()
        rclpy.shutdown()
```

**Run:**
```bash
colcon test --packages-select manipulation_bringup
```

### Simulation Tests

End-to-end tests in simulation environment.

**Example:** `tests/integration/test_pick_place.py`

```python
import rclpy
from rclpy.node import Node
from manipulation_msgs.action import ManipulationTask
from rclpy.action import ActionClient

class TestPickPlace(Node):
    def __init__(self):
        super().__init__('test_pick_place')
        self.action_client = ActionClient(
            self, ManipulationTask, '/manipulation/execute_task'
        )

    def test_pick_object(self):
        """Test picking up an object at known pose."""
        goal = ManipulationTask.Goal()
        goal.task_description = "Pick up red cube"
        goal.target_object_pose = ...  # Known pose in simulation

        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(goal)

        # Wait for result
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        assert result.success == True
```

## Test Organization

### Per-Package Tests

Each ROS package can have its own `test/` directory:

```
src/manipulation_adapter/
├── src/
├── include/
├── test/
│   ├── test_adapter.cpp
│   └── test_safety.cpp
└── CMakeLists.txt  # Includes test targets
```

**CMakeLists.txt example:**
```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_adapter test/test_adapter.cpp)
  target_link_libraries(test_adapter ${PROJECT_NAME})

  ament_add_gtest(test_safety test/test_safety.cpp)
  target_link_libraries(test_safety ${PROJECT_NAME})
endif()
```

### Centralized Integration Tests

Use `tests/integration/` for tests that span multiple packages:

```
tests/integration/
├── test_core_launch.py      # Test launch files
├── test_communication.py    # Test inter-node communication
├── test_sim_manipulation.py # Test in simulation
└── fixtures/                # Shared test fixtures
    ├── test_worlds/
    └── test_models/
```

## Continuous Integration

GitHub Actions runs tests automatically on every PR:

```yaml
# .github/workflows/ci.yml
- name: Run tests
  run: |
    source install/setup.bash
    colcon test --event-handlers console_direct+
    colcon test-result --verbose
```

## Test Coverage Goals

| Component | Target Coverage |
|-----------|----------------|
| Adapter (safety) | 90% |
| Policy (inference) | 70% |
| Perception | 80% |
| Overall | 75% |

## Writing Good Tests

### Principles

1. **Isolated**: Each test is independent
2. **Repeatable**: Same input → same output
3. **Fast**: Unit tests < 1s, integration < 30s
4. **Clear**: Test name describes what it verifies

### Test Naming

```
test_<component>_<scenario>_<expected_result>
```

Examples:
- `test_adapter_joint_limit_rejected`
- `test_policy_valid_input_returns_action`
- `test_perception_missing_image_handles_gracefully`

### Fixtures and Mocks

Use fixtures for common setups:

```python
# conftest.py
@pytest.fixture
def mock_camera_image():
    """Provide a standard test image."""
    return np.zeros((480, 640, 3), dtype=np.uint8)

@pytest.fixture
def mock_joint_states():
    """Provide standard joint states."""
    msg = JointState()
    msg.name = ['joint1', 'joint2', 'joint3']
    msg.position = [0.0, 0.5, 1.0]
    return msg
```

## Debugging Tests

### Run in Verbose Mode

```bash
colcon test --event-handlers console_direct+ --packages-select <package>
```

### Run Single Test

```bash
# Python
pytest src/manipulation_policy/test/test_policy.py::test_specific_function -v

# C++ (after build)
./build/manipulation_adapter/test_adapter --gtest_filter=SafetyTest.JointLimitCheck
```

### Use pdb for Python

```python
def test_something():
    import pdb; pdb.set_trace()  # Debugger
    result = function_under_test()
    assert result == expected
```

## Best Practices

1. **Test Before Merge**: All PRs must pass CI tests
2. **Test Edge Cases**: Not just happy path
3. **Use Assertions**: Clear, specific assertions
4. **Mock External Dependencies**: Isolate unit tests
5. **Clean Up**: Restore state after tests
6. **Document Complex Tests**: Explain non-obvious logic

## Resources

- [GTest Documentation](https://google.github.io/googletest/)
- [pytest Documentation](https://docs.pytest.org/)
- [ROS 2 Testing](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Testing-Main.html)
- [launch_testing](https://github.com/ros2/launch/tree/rolling/launch_testing)

---

*Write tests, catch bugs early, ship with confidence!* ✅
