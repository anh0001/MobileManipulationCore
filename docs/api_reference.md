# API Reference

This document describes the ROS 2 interfaces (topics, services, actions, messages) provided by the MobileManipulationCore stack.

## Table of Contents

1. [Custom Messages](#custom-messages)
2. [Custom Actions](#custom-actions)
3. [Topics](#topics)
4. [Action Servers/Clients](#action-servers-clients)
5. [Parameters](#parameters)

## Custom Messages

### manipulation_msgs/PolicyOutput

High-level manipulation commands from the policy model.

**Fields:**
```
std_msgs/Header header

# End-effector target
geometry_msgs/Pose eef_target_pose
bool has_eef_target

# Alternative: joint angle deltas
float64[] joint_deltas
bool has_joint_deltas

# Gripper command (0.0 = closed, 1.0 = open)
float64 gripper_command
bool gripper_active

# Optional base motion hint
geometry_msgs/Twist base_velocity_hint
bool has_base_hint

# Reference frame for targets
string reference_frame

# Policy confidence (0.0 to 1.0)
float32 confidence
```

**Usage:**
Published by `policy_node`, subscribed by `adapter_node`.

---

## Custom Actions

### manipulation_msgs/action/ManipulationTask

High-level manipulation task execution.

**Goal:**
```
string task_description          # Natural language description
geometry_msgs/PoseStamped target_object_pose  # Optional target
bool use_policy                  # Use VLA policy vs hardcoded
```

**Result:**
```
bool success
string message                   # Status/error message
float32 execution_time          # Seconds
```

**Feedback:**
```
string current_state            # e.g., "approaching", "grasping"
float32 progress                # 0.0 to 1.0
geometry_msgs/PoseStamped current_eef_pose
```

**Usage:**
Provided by `adapter_node` (future implementation).

---

## Topics

### Subscribed Topics (Inputs from Base)

#### /camera/image_raw
- **Type:** `sensor_msgs/msg/Image`
- **Description:** RGB camera feed for visual input
- **Subscribers:** `perception_node`
- **QoS:** Best effort, depth 10

#### /camera/depth/image_raw
- **Type:** `sensor_msgs/msg/Image`
- **Description:** Depth image aligned with RGB
- **Subscribers:** `perception_node`
- **QoS:** Best effort, depth 10

#### /joint_states
- **Type:** `sensor_msgs/msg/JointState`
- **Description:** Current joint positions, velocities, efforts
- **Subscribers:** `perception_node`, `adapter_node`
- **QoS:** Reliable, depth 10

#### /tf and /tf_static
- **Type:** `tf2_msgs/msg/TFMessage`
- **Description:** Transform tree
- **Subscribers:** All nodes (via TF2)
- **Frames Expected:**
  - `map` → `odom` → `base_link`
  - `base_link` → arm links → `gripper_link`
  - `base_link` → `camera_link` → `camera_optical_frame`

#### /odom
- **Type:** `nav_msgs/msg/Odometry`
- **Description:** Base odometry
- **Subscribers:** `adapter_node` (optional)
- **QoS:** Reliable, depth 10

### Published Topics (Commands to Hardware)

#### /cmd_vel
- **Type:** `geometry_msgs/msg/Twist`
- **Description:** Velocity commands for mobile base
- **Publishers:** `adapter_node`
- **QoS:** Reliable, depth 1 (keep last)
- **Rates:** 10-50 Hz depending on configuration

#### /manipulation/policy_output
- **Type:** `manipulation_msgs/msg/PolicyOutput`
- **Description:** High-level actions from policy
- **Publishers:** `policy_node`
- **Subscribers:** `adapter_node`
- **QoS:** Reliable, depth 10
- **Rates:** Configurable, typically 5-10 Hz

---

## Action Servers/Clients

### Action Clients (Used by Our Stack)

#### /navigate_to_pose
- **Type:** `nav2_msgs/action/NavigateToPose`
- **Description:** Navigate base to target pose
- **Client:** `adapter_node`
- **Server:** Nav2 (from base stack)

#### /arm_controller/follow_joint_trajectory
- **Type:** `control_msgs/action/FollowJointTrajectory`
- **Description:** Execute arm joint trajectory
- **Client:** `adapter_node`
- **Server:** Piper driver (from base stack)

#### /move_action
- **Type:** `moveit_msgs/action/MoveGroup`
- **Description:** Plan and execute arm motions via MoveIt
- **Client:** `adapter_node`
- **Server:** MoveIt `move_group`

### Action Servers (Provided by Our Stack)

#### /manipulation/execute_task
- **Type:** `manipulation_msgs/action/ManipulationTask`
- **Description:** Execute high-level manipulation task
- **Server:** `adapter_node` (future)
- **Status:** Planned for future release

---

## Parameters

### perception_node

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `camera_topic` | string | `/camera/color/image_raw` | RGB camera topic |
| `depth_topic` | string | `/camera/depth/image_raw` | Depth camera topic |
| `joint_states_topic` | string | `/joint_states` | Joint states topic |
| `image_width` | int | 224 | Resize width for policy |
| `image_height` | int | 224 | Resize height for policy |

**Example:**
```bash
ros2 run manipulation_perception perception_node \
  --ros-args -p camera_topic:=/my_camera/image
```

### policy_node

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model_name` | string | `openvla-7b` | Model identifier |
| `model_path` | string | `` | Local model path (empty = download) |
| `use_remote` | bool | `false` | Use remote inference server |
| `remote_url` | string | `http://localhost:5000` | Remote server URL |
| `remote_timeout_sec` | double | 1.0 | Remote inference timeout (seconds) |
| `remote_retry_attempts` | int | 3 | Remote retry attempts |
| `remote_fallback_on_failure` | bool | `true` | Fallback to stub when remote fails |
| `inference_rate` | double | 10.0 | Inference frequency (Hz) |
| `device` | string | `cuda` | Compute device (cuda/cpu) |
| `use_fp16` | bool | `true` | Use half-precision |

**Example:**
```bash
ros2 run manipulation_policy policy_node \
  --ros-args \
  -p use_remote:=true \
  -p remote_url:=http://192.168.1.100:5000 \
  -p inference_rate:=15.0
```

### adapter_node

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `base_frame` | string | `base_link` | Base reference frame |
| `ee_frame` | string | `piper_gripper_base` | End-effector frame |
| `joint_states_topic` | string | `/joint_states` | Joint states topic |
| `navigate_to_pose_action` | string | `/navigate_to_pose` | Nav2 action name |
| `follow_joint_trajectory_action` | string | `/arm_controller/follow_joint_trajectory` | Arm trajectory action name |
| `gripper_follow_joint_trajectory_action` | string | `/gripper_controller/follow_joint_trajectory` | Gripper trajectory action name |
| `use_moveit` | bool | true | Use MoveIt for EEF pose targets |
| `move_group_action` | string | `/move_action` | MoveIt MoveGroup action name |
| `move_group_name` | string | `arm` | MoveIt planning group |
| `move_group_eef_link` | string | `piper_link6` | Link constrained for MoveIt goals |
| `moveit_action_wait_sec` | double | 1.0 | MoveIt action wait time (s) |
| `moveit_planning_time` | double | 2.0 | Planning time (s) |
| `moveit_planning_attempts` | int | 3 | Planning attempts |
| `moveit_velocity_scaling` | double | 0.5 | Velocity scaling |
| `moveit_accel_scaling` | double | 0.5 | Acceleration scaling |
| `moveit_position_tolerance` | double | 0.01 | Position tolerance (m) |
| `moveit_orientation_tolerance` | double | 0.1 | Orientation tolerance (rad) |
| `arm_joint_names` | string[] | `[]` | Joint name order for joint deltas |
| `arm_command_duration_sec` | double | 1.5 | Arm trajectory duration (s) |
| `gripper_joint_name` | string | `piper_joint7` | Gripper joint name |
| `gripper_joint_names` | string[] | `[]` | Gripper joint names (multi-joint) |
| `gripper_open_position` | double | 0.035 | Gripper open position (rad/m) |
| `gripper_closed_position` | double | 0.0 | Gripper closed position (rad/m) |
| `gripper_open_positions` | double[] | `[]` | Gripper open positions per joint |
| `gripper_closed_positions` | double[] | `[]` | Gripper closed positions per joint |
| `gripper_command_duration_sec` | double | 0.75 | Gripper trajectory duration (s) |
| `gripper_command_epsilon` | double | 0.01 | Minimum change to resend gripper command |
| `max_base_velocity` | double | 0.5 | Max base speed (m/s) |
| `max_arm_velocity` | double | 1.0 | Max joint velocity (rad/s) |
| `safety_timeout_sec` | double | 2.0 | Policy timeout (seconds) |
| `workspace_x_min` | double | 0.1 | Workspace boundary (m) |
| `workspace_x_max` | double | 0.8 | Workspace boundary (m) |
| `workspace_y_min` | double | -0.5 | Workspace boundary (m) |
| `workspace_y_max` | double | 0.5 | Workspace boundary (m) |
| `workspace_z_min` | double | 0.0 | Workspace boundary (m) |
| `workspace_z_max` | double | 1.0 | Workspace boundary (m) |

**Example:**
```bash
ros2 run manipulation_adapter adapter_node \
  --ros-args \
  -p max_base_velocity:=0.3 \
  -p safety_timeout_sec:=1.5
```

---

## Message Flows

### Normal Operation Flow

```
                    ┌──────────────┐
                    │   Hardware   │
                    │   & Sensors  │
                    └──────┬───────┘
                           │
            ┌──────────────┴──────────────┐
            │                             │
      /joint_states               /camera/image
            │                             │
            ▼                             ▼
    ┌───────────────┐          ┌──────────────────┐
    │  Perception   │          │  Perception      │
    │  (joints)     │          │  (vision)        │
    └───────┬───────┘          └────────┬─────────┘
            │                           │
            └─────────┬─────────────────┘
                      │
                      ▼
              ┌───────────────┐
              │  Policy Node  │
              │  (VLA Model)  │
              └───────┬───────┘
                      │
            /manipulation/policy_output
                      │
                      ▼
            ┌─────────────────┐
            │  Adapter Node   │
            │  (Coordinator)  │
            └────────┬────────┘
                     │
         ┌───────────┼───────────┐
         │           │           │
    /cmd_vel    /arm_traj   /gripper_cmd
         │           │           │
         ▼           ▼           ▼
    ┌────────────────────────────┐
    │  Hardware Controllers      │
    └────────────────────────────┘
```

### Split Deployment Flow

```
  Robot (Jetson)                   Remote Server (GPU)
  ┌─────────────┐                  ┌─────────────────┐
  │ Perception  │─────(image)─────>│  Policy Node    │
  │    Node     │                  │  (VLA Inference)│
  └─────────────┘<────(action)─────└─────────────────┘
         │                                   │
         │                            ROS2 network
         │                            or gRPC/REST
         ▼
  ┌─────────────┐
  │   Adapter   │
  │    Node     │
  └─────────────┘
```

---

## Usage Examples

### Python: Subscribe to Policy Output

```python
import rclpy
from rclpy.node import Node
from manipulation_msgs.msg import PolicyOutput

class PolicyListener(Node):
    def __init__(self):
        super().__init__('policy_listener')
        self.sub = self.create_subscription(
            PolicyOutput,
            '/manipulation/policy_output',
            self.callback,
            10
        )

    def callback(self, msg):
        if msg.has_eef_target:
            print(f"EEF target: {msg.eef_target_pose.position}")
        if msg.gripper_active:
            print(f"Gripper: {msg.gripper_command}")

def main():
    rclpy.init()
    node = PolicyListener()
    rclpy.spin(node)
    rclpy.shutdown()
```

### C++: Publish Base Command

```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class BaseCommander : public rclcpp::Node {
public:
  BaseCommander() : Node("base_commander") {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

  void sendCommand(double linear_x, double angular_z) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_x;
    cmd.angular.z = angular_z;
    pub_->publish(cmd);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};
```

### Launch File: Custom Configuration

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='manipulation_policy',
            executable='policy_node',
            name='policy_node',
            parameters=[{
                'model_name': 'openvla-1b',
                'inference_rate': 15.0,
                'use_fp16': True
            }]
        )
    ])
```

---

## Coordinate Frames

### Standard Frame Convention

- **map**: Global fixed frame
- **odom**: Local odometry frame (drifts over time)
- **base_link**: Robot base center, moves with robot
- **base_footprint**: Projection of base_link onto ground
- **camera_link**: Camera mount point
- **camera_optical_frame**: Camera optical center (ROS convention: +X right, +Y down, +Z forward)
- **piper_link_0**: Arm base (coincident with base_link)
- **piper_link_1..6**: Arm links
- **piper_gripper_base**: End-effector (gripper center point)

### Transform Relationships

```
map
 └─ odom
     └─ base_link
         ├─ camera_link
         │   └─ camera_optical_frame
         └─ piper_link_0
             └─ piper_link_1
                 └─ ...
                     └─ piper_gripper_base
```

---

## Troubleshooting

### Common Issues

**Topic not publishing:**
- Check `ros2 topic list`
- Verify node is running: `ros2 node list`
- Check topic type: `ros2 topic info <topic_name>`

**TF lookup fails:**
- View frames: `ros2 run tf2_tools view_frames`
- Check transform: `ros2 run tf2_ros tf2_echo <source> <target>`

**Action server not available:**
- List action servers: `ros2 action list`
- Check server info: `ros2 action info <action_name>`

---

## Further Information

- **ROS 2 Humble Documentation:** https://docs.ros.org/en/humble/
- **TF2 Tutorials:** https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html
- **Action Documentation:** https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html

---

*For more examples, see the `examples/` directory (coming soon).*
