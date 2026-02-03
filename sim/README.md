# Simulation Assets

This directory contains simulation-related files for testing the manipulation stack in Gazebo or other simulators.

## Directory Structure

```
sim/
├── urdf/           # Robot URDF/Xacro descriptions
├── worlds/         # Gazebo world files
└── launch/         # Simulation launch files
```

## Usage

### Launching Simulation

```bash
ros2 launch manipulation_bringup sim_launch.py
```

Or directly:

```bash
ros2 launch sim/launch/gazebo_launch.py
```

## URDF Files

URDF (Unified Robot Description Format) files describe the robot's physical structure:
- Link geometries
- Joint definitions
- Visual and collision meshes
- Sensors (camera, IMU, etc.)

### Creating Your Robot URDF

1. If you have an existing URDF from your base repository, link or copy it here
2. Ensure it includes both the mobile base and manipulator arm
3. Add camera and sensor definitions if not already present

Example structure:
```xml
<robot name="mobile_manipulator">
  <!-- Mobile base -->
  <link name="base_link">...</link>

  <!-- Manipulator arm -->
  <link name="arm_link_1">...</link>
  <!-- ... more arm links ... -->

  <!-- Sensors -->
  <link name="camera_link">...</link>
  <gazebo reference="camera_link">
    <!-- Camera sensor definition -->
  </gazebo>
</robot>
```

## World Files

Gazebo worlds define the simulation environment:
- Ground plane
- Obstacles and objects
- Lighting
- Physics properties

### Available Worlds

- `empty_world.sdf` - Minimal environment for basic testing
- `manipulation_test.sdf` - World with objects for pick-and-place testing
- *(Add your custom worlds here)*

### Creating Custom Worlds

1. Use Gazebo's Building Editor or write SDF manually
2. Add objects with known poses for manipulation testing
3. Include markers or targets for navigation

## Launch Files

Simulation launch files coordinate starting Gazebo and robot controllers.

### gazebo_launch.py

Basic structure:
```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot', '-file', 'path/to/urdf']
    )

    return LaunchDescription([gazebo, spawn_robot])
```

## Integration with Base Stack

If your base repository (e.g., ranger-garden-assistant) already provides simulation:

1. **Option A: Use Their Sim**
   - Launch their Gazebo setup
   - Our manipulation stack connects via ROS topics
   - No need to duplicate URDF/worlds

2. **Option B: Extended Sim**
   - Start with their URDF as base
   - Add manipulation-specific test environments here
   - Useful for isolated manipulation testing

## Testing Scenarios

Create test scenarios for validation:

1. **Static Object Pick**: Object at known pose
2. **Dynamic Target**: Moving object tracking
3. **Navigation + Manipulation**: Combine base motion with arm control
4. **Collision Avoidance**: Obstacles in workspace

Example test world setup:
```xml
<sdf version="1.6">
  <world name="manipulation_test">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <model name="test_object">
      <pose>0.5 0 0.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box><size>0.05 0.05 0.05</size></box>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <box><size>0.05 0.05 0.05</size></box>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

## Debugging Tips

- **Visualize TF**: `ros2 run tf2_tools view_frames`
- **Check joints**: `ros2 topic echo /joint_states`
- **Monitor camera**: `ros2 run rqt_image_view rqt_image_view`
- **Gazebo GUI**: Launch with GUI enabled to visually debug

## Resources

- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [URDF in ROS 2](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs)

---

*Note: Simulation setup is optional but highly recommended for safe testing before hardware deployment.*
