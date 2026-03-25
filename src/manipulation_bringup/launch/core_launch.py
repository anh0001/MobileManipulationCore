#!/usr/bin/env python3
# Copyright 2026 MobileManipulationCore Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Launch file for the core manipulation intelligence stack.

This launch file starts all components needed for mobile manipulation
on the real robot (Jetson-only deployment).
"""

import math
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    def load_yaml(path):
        try:
            with open(path, 'r', encoding='utf-8') as handle:
                return yaml.safe_load(handle) or {}
        except FileNotFoundError:
            return {}

    def resolve_config_path(filename):
        pkg_share = get_package_share_directory('manipulation_bringup')
        candidate = os.path.join(pkg_share, 'config', filename)
        if os.path.isfile(candidate):
            return candidate
        fallback = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', '..', '..', 'config', filename)
        )
        return fallback

    def normalize_bounds(value, default):
        if not isinstance(value, (list, tuple)) or len(value) != 2:
            return [default[0], default[1]]
        try:
            lower = float(value[0])
            upper = float(value[1])
        except (TypeError, ValueError):
            return [default[0], default[1]]
        if not math.isfinite(lower) or not math.isfinite(upper) or lower > upper:
            return [default[0], default[1]]
        return [lower, upper]

    def normalize_quaternion(qx, qy, qz, qw):
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 1e-12:
            return 0.0, 0.0, 0.0, 1.0
        inv = 1.0 / norm
        return qx * inv, qy * inv, qz * inv, qw * inv

    def rpy_to_quaternion(roll, pitch, yaw):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return normalize_quaternion(qx, qy, qz, qw)

    def quaternion_multiply(lhs, rhs):
        lx, ly, lz, lw = lhs
        rx, ry, rz, rw = rhs
        return (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )

    def rotate_vector(q, vec):
        vx, vy, vz = vec
        qx, qy, qz, qw = normalize_quaternion(*q)
        q_vec = (vx, vy, vz, 0.0)
        q_conj = (-qx, -qy, -qz, qw)
        rotated = quaternion_multiply(quaternion_multiply((qx, qy, qz, qw), q_vec), q_conj)
        return rotated[0], rotated[1], rotated[2]

    def invert_transform(tx, ty, tz, roll, pitch, yaw):
        qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)
        q_inv = (-qx, -qy, -qz, qw)
        rx, ry, rz = rotate_vector(q_inv, (tx, ty, tz))
        return -rx, -ry, -rz, q_inv[0], q_inv[1], q_inv[2], q_inv[3]

    robot_cfg = load_yaml(resolve_config_path('robot_params.yaml'))
    policy_cfg_full = load_yaml(resolve_config_path('policy_params.yaml'))

    robot_topics = robot_cfg.get('robot', {}).get('topics', {})
    robot_frames = robot_cfg.get('robot', {}).get('frames', {})
    robot_actions = robot_cfg.get('robot', {}).get('actions', {})
    base_cfg = robot_cfg.get('base', {})
    arm_cfg = robot_cfg.get('arm', {})
    gripper_cfg = robot_cfg.get('gripper', {})
    moveit_cfg = robot_cfg.get('moveit', {})
    workspace_cfg = robot_cfg.get('workspace', {})
    bridge_virtual_cfg = robot_cfg.get('bridge_v2_virtual_frames', {})

    policy_cfg = policy_cfg_full.get('policy', {})
    action_cfg = policy_cfg_full.get('action', {})
    remote_cfg = policy_cfg_full.get('remote', {})
    safety_cfg = policy_cfg_full.get('safety', {})
    observation_cfg = policy_cfg_full.get('observation', {})
    action_bounds_cfg = policy_cfg.get('action_bounds', {})
    openvla_clip_actions = bool(policy_cfg.get('clip_actions', False))
    openvla_position_bounds = normalize_bounds(
        action_bounds_cfg.get('position', [-1.0, 1.0]),
        (-1.0, 1.0),
    )
    openvla_rotation_bounds = normalize_bounds(
        action_bounds_cfg.get('rotation', [-math.pi, math.pi]),
        (-math.pi, math.pi),
    )
    openvla_gripper_bounds = normalize_bounds(
        action_bounds_cfg.get('gripper', [0.0, 1.0]),
        (0.0, 1.0),
    )

    image_size = policy_cfg.get('image_size', [224, 224])
    image_width = image_size[0] if isinstance(image_size, (list, tuple)) and len(image_size) > 0 else 224
    image_height = image_size[1] if isinstance(image_size, (list, tuple)) and len(image_size) > 1 else 224
    observation_topic = observation_cfg.get('topic', '/manipulation/observation')
    inference_rate_hz = float(policy_cfg.get('inference_rate', 10.0))
    configured_safety_timeout_sec = float(safety_cfg.get('timeout_sec', 2.0))

    # Keep adapter safety timeout comfortably above expected policy output intervals.
    # This avoids canceling active goals when inference runs slowly (e.g., 0.5 Hz remote OpenVLA).
    effective_safety_timeout_sec = configured_safety_timeout_sec
    if inference_rate_hz > 0.0:
        effective_safety_timeout_sec = max(effective_safety_timeout_sec, 3.0 / inference_rate_hz)

    # Remote inference callbacks can block up to timeout * retries. Account for that worst case.
    remote_timeout_sec = float(remote_cfg.get('timeout', 1.0))
    remote_retry_attempts = int(remote_cfg.get('retry_attempts', 3))
    worst_case_remote_cycle_sec = remote_timeout_sec * max(1, remote_retry_attempts)
    effective_safety_timeout_sec = max(effective_safety_timeout_sec, worst_case_remote_cycle_sec + 1.0)

    default_arm_execution_mode = str(
        moveit_cfg.get('arm_execution_mode', 'moveit_servo')
    ).strip() or 'moveit_servo'
    configured_servo_horizon_sec = float(moveit_cfg.get('servo_command_horizon_sec', 0.0))
    if configured_servo_horizon_sec > 0.0:
        effective_servo_horizon_sec = configured_servo_horizon_sec
    elif inference_rate_hz > 0.0:
        effective_servo_horizon_sec = 1.0 / inference_rate_hz
    else:
        effective_servo_horizon_sec = 2.0

    bridge_virtual_enabled = bool(bridge_virtual_cfg.get('enabled', False))
    bridge_parent_camera_frame = str(
        bridge_virtual_cfg.get('parent_camera_frame', robot_frames.get('camera_link', ''))
    )
    bridge_camera_virtual_frame = str(
        bridge_virtual_cfg.get('camera_virtual_frame', 'bridge_v2_camera_virtual')
    )
    bridge_base_virtual_frame = str(
        bridge_virtual_cfg.get('base_virtual_frame', 'bridge_v2_base_virtual')
    )

    camera_in_bridge_base_cfg = bridge_virtual_cfg.get('camera_in_bridge_base', {})
    bridge_cam_x = float(camera_in_bridge_base_cfg.get('x', -0.18))
    bridge_cam_y = float(camera_in_bridge_base_cfg.get('y', 0.0))
    bridge_cam_z = float(camera_in_bridge_base_cfg.get('z', 0.50))
    bridge_cam_roll = float(camera_in_bridge_base_cfg.get('roll', 0.0))
    bridge_cam_pitch = float(camera_in_bridge_base_cfg.get('pitch', -0.78539816339))
    bridge_cam_yaw = float(camera_in_bridge_base_cfg.get('yaw', 0.0))

    bridge_base_from_camera = invert_transform(
        bridge_cam_x,
        bridge_cam_y,
        bridge_cam_z,
        bridge_cam_roll,
        bridge_cam_pitch,
        bridge_cam_yaw,
    )
    configured_policy_reference_frame = str(policy_cfg.get('reference_frame', '')).strip()
    policy_reference_frame = (
        bridge_base_virtual_frame if bridge_virtual_enabled else configured_policy_reference_frame
    )
    if bridge_virtual_enabled and (
        not bridge_parent_camera_frame
        or not bridge_camera_virtual_frame
        or not bridge_base_virtual_frame
    ):
        bridge_virtual_enabled = False
        policy_reference_frame = configured_policy_reference_frame

    # Declare launch arguments
    use_remote_policy_arg = DeclareLaunchArgument(
        'use_remote_policy',
        default_value=str(remote_cfg.get('enabled', False)).lower(),
        description='Use remote policy server for inference'
    )

    remote_url_arg = DeclareLaunchArgument(
        'remote_url',
        default_value=remote_cfg.get('server_url', 'http://localhost:5000'),
        description='URL of remote policy server'
    )

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value=robot_topics.get('camera_rgb', '/camera/color/image_raw'),
        description='Camera topic for visual input'
    )

    arm_execution_mode_arg = DeclareLaunchArgument(
        'arm_execution_mode',
        default_value=default_arm_execution_mode,
        description='Arm execution mode for adapter_node: moveit_servo or move_group',
    )

    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='vla',
        description='High-level control mode: vla or visual_servo',
    )

    # Get launch configurations
    use_remote_policy = LaunchConfiguration('use_remote_policy')
    remote_url = LaunchConfiguration('remote_url')
    camera_topic = LaunchConfiguration('camera_topic')
    arm_execution_mode = LaunchConfiguration('arm_execution_mode')
    control_mode = LaunchConfiguration('control_mode')
    joint_states_topic = robot_topics.get('joint_states', '/joint_states')

    # Conditions for control mode selection
    is_vla_mode = IfCondition(
        PythonExpression(["'", control_mode, "' == 'vla'"])
    )
    is_visual_servo_mode = IfCondition(
        PythonExpression(["'", control_mode, "' == 'visual_servo'"])
    )

    # Load visual servo config
    vs_cfg_full = load_yaml(resolve_config_path('visual_servo_params.yaml'))
    vs_cfg = vs_cfg_full.get('visual_servo', {})
    vs_control_cfg = vs_cfg.get('control', {})
    vs_debug_cfg = vs_cfg.get('debug', {})
    detection_cfg_full = load_yaml(resolve_config_path('detection_params.yaml'))
    detection_cfg = detection_cfg_full.get('detection', {})

    # Perception node
    perception_node = Node(
        package='manipulation_perception',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[{
            'camera_topic': camera_topic,
            'depth_topic': robot_topics.get('camera_depth', '/camera/depth/image_raw'),
            'joint_states_topic': robot_topics.get('joint_states', '/joint_states'),
            'image_width': int(image_width),
            'image_height': int(image_height),
            'observation_topic': observation_topic,
            'base_frame': robot_frames.get('base_link', 'base_link'),
            'camera_frame': robot_frames.get('camera_link', ''),
            'ee_frame': robot_frames.get('ee_link', ''),
            'include_image': bool(observation_cfg.get('include_image', True)),
            'include_depth': bool(observation_cfg.get('include_depth', False)),
            'include_joint_states': bool(observation_cfg.get('include_joint_states', True)),
            'sync_tolerance_sec': float(observation_cfg.get('sync_tolerance_sec', 0.05)),
        }]
    )

    bridge_virtual_nodes = []
    if bridge_virtual_enabled:
        inv_x, inv_y, inv_z, inv_qx, inv_qy, inv_qz, inv_qw = bridge_base_from_camera
        base_parent_frame = bridge_camera_virtual_frame
        if bridge_camera_virtual_frame != bridge_parent_camera_frame:
            bridge_virtual_nodes.append(
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='bridge_v2_camera_virtual_tf',
                    output='screen',
                    arguments=[
                        '0.0', '0.0', '0.0',
                        '0.0', '0.0', '0.0', '1.0',
                        bridge_parent_camera_frame,
                        bridge_camera_virtual_frame,
                    ],
                )
            )
        else:
            base_parent_frame = bridge_parent_camera_frame

        bridge_virtual_nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='bridge_v2_base_virtual_tf',
                output='screen',
                arguments=[
                    str(inv_x), str(inv_y), str(inv_z),
                    str(inv_qx), str(inv_qy), str(inv_qz), str(inv_qw),
                    base_parent_frame,
                    bridge_base_virtual_frame,
                ],
            )
        )

    # Policy node (local) - only in VLA mode
    policy_node_local = Node(
        package='manipulation_policy',
        executable='policy_node',
        name='policy_node',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", control_mode, "' == 'vla' and '",
            use_remote_policy, "' != 'true'"
        ])),
        parameters=[{
            'model_name': policy_cfg.get('model_name', 'openvla-7b'),
            'model_path': policy_cfg.get('model_path', ''),
            'use_remote': False,
            'inference_rate': float(policy_cfg.get('inference_rate', 10.0)),
            'max_steps': int(policy_cfg.get('max_steps', 50)),
            'device': policy_cfg.get('device', 'cuda'),
            'use_fp16': bool(policy_cfg.get('use_fp16', True)),
            'remote_timeout_sec': float(remote_cfg.get('timeout', 1.0)),
            'remote_retry_attempts': int(remote_cfg.get('retry_attempts', 3)),
            'remote_fallback_on_failure': bool(remote_cfg.get('fallback_on_failure', False)),
            'camera_topic': camera_topic,
            'camera_frame': robot_frames.get('camera_link', ''),
            'arm_base_frame': robot_frames.get('arm_base', 'piper_base_link'),
            'reference_frame': policy_reference_frame,
            'joint_states_topic': joint_states_topic,
            'use_observation': True,
            'observation_topic': observation_topic,
            'task_prompt_topic': policy_cfg.get('task_prompt_topic', '/manipulation/task_prompt'),
            'openvla_xyz_scaling': float(policy_cfg.get('action_scaling_xyz', 1.0)),
            'openvla_rotation_scaling': float(policy_cfg.get('action_scaling_rotation', 1.0)),
            'openvla_clip_actions': openvla_clip_actions,
            'openvla_position_bounds': openvla_position_bounds,
            'openvla_rotation_bounds': openvla_rotation_bounds,
            'openvla_gripper_bounds': openvla_gripper_bounds,
        }]
    )

    # Policy node (remote client) - only in VLA mode
    policy_node_remote = Node(
        package='manipulation_policy',
        executable='policy_node',
        name='policy_node',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", control_mode, "' == 'vla' and '",
            use_remote_policy, "' == 'true'"
        ])),
        parameters=[{
            'use_remote': True,
            'remote_url': remote_url,
            'inference_rate': float(policy_cfg.get('inference_rate', 10.0)),
            'max_steps': int(policy_cfg.get('max_steps', 50)),
            'device': policy_cfg.get('device', 'cuda'),
            'use_fp16': bool(policy_cfg.get('use_fp16', True)),
            'remote_timeout_sec': float(remote_cfg.get('timeout', 1.0)),
            'remote_retry_attempts': int(remote_cfg.get('retry_attempts', 3)),
            'remote_fallback_on_failure': bool(remote_cfg.get('fallback_on_failure', False)),
            'camera_topic': camera_topic,
            'camera_frame': robot_frames.get('camera_link', ''),
            'arm_base_frame': robot_frames.get('arm_base', 'piper_base_link'),
            'reference_frame': policy_reference_frame,
            'joint_states_topic': joint_states_topic,
            'use_observation': True,
            'observation_topic': observation_topic,
            'task_prompt_topic': policy_cfg.get('task_prompt_topic', '/manipulation/task_prompt'),
            'openvla_xyz_scaling': float(policy_cfg.get('action_scaling_xyz', 1.0)),
            'openvla_rotation_scaling': float(policy_cfg.get('action_scaling_rotation', 1.0)),
            'openvla_clip_actions': openvla_clip_actions,
            'openvla_position_bounds': openvla_position_bounds,
            'openvla_rotation_bounds': openvla_rotation_bounds,
            'openvla_gripper_bounds': openvla_gripper_bounds,
        }]
    )

    # Adapter node
    adapter_node = Node(
        package='manipulation_adapter',
        executable='adapter_node',
        name='adapter_node',
        output='screen',
        parameters=[{
            'base_frame': robot_frames.get('base_link', 'base_link'),
            'ee_frame': robot_frames.get('ee_link', 'piper_tcp'),
            'joint_states_topic': robot_topics.get('joint_states', '/joint_states'),
            'navigate_to_pose_action': robot_actions.get('navigate_to_pose', '/navigate_to_pose'),
            'follow_joint_trajectory_action': robot_actions.get(
                'follow_joint_trajectory', '/arm_controller/follow_joint_trajectory'
            ),
            'use_moveit': bool(moveit_cfg.get('enabled', False)),
            'move_group_action': moveit_cfg.get('move_group_action', '/move_action'),
            'move_group_name': moveit_cfg.get('move_group_name', 'arm'),
            'move_group_eef_link': moveit_cfg.get('eef_link', 'piper_tcp'),
            'gripper_move_group_name': moveit_cfg.get(
                'gripper_move_group_name', 'piper_gripper'
            ),
            'gripper_move_group_timeout_sec': float(
                moveit_cfg.get('gripper_move_group_timeout_sec', 15.0)
            ),
            'moveit_action_wait_sec': float(moveit_cfg.get('action_wait_sec', 1.0)),
            'moveit_planning_time': float(moveit_cfg.get('planning_time', 2.0)),
            'moveit_planning_attempts': int(moveit_cfg.get('planning_attempts', 3)),
            'moveit_velocity_scaling': float(moveit_cfg.get('velocity_scaling', 0.5)),
            'moveit_accel_scaling': float(moveit_cfg.get('accel_scaling', 0.5)),
            'moveit_position_tolerance': float(moveit_cfg.get('position_tolerance', 0.01)),
            'moveit_orientation_tolerance': float(moveit_cfg.get('orientation_tolerance', 0.1)),
            'move_to_ready_on_startup': bool(
                moveit_cfg.get('move_to_ready_on_startup', True)
            ),
            'ready_pose_joint_names': moveit_cfg.get(
                'ready_pose_joint_names',
                [
                    'piper_joint1',
                    'piper_joint2',
                    'piper_joint3',
                    'piper_joint4',
                    'piper_joint5',
                    'piper_joint6',
                ],
            ),
            'ready_pose_joint_positions': moveit_cfg.get(
                'ready_pose_joint_positions',
                [0.0, 1.2, -0.2, 0.0, -0.8, 0.0],
            ),
            'ready_pose_start_delay_sec': float(
                moveit_cfg.get('ready_pose_start_delay_sec', 1.0)
            ),
            'ready_pose_retry_period_sec': float(
                moveit_cfg.get('ready_pose_retry_period_sec', 1.0)
            ),
            'ready_pose_max_attempts': int(
                moveit_cfg.get('ready_pose_max_attempts', 3)
            ),
            'arm_joint_names': arm_cfg.get('joint_names', []),
            'arm_command_duration_sec': float(arm_cfg.get('command_duration_sec', 1.5)),
            'gripper_joint_name': gripper_cfg.get('joint_name', 'piper_joint7'),
            'gripper_joint_names': gripper_cfg.get('joint_names', []),
            'gripper_open_position': float(gripper_cfg.get('open_position', 0.065)),
            'gripper_closed_position': float(gripper_cfg.get('closed_position', 0.0)),
            'gripper_open_positions': gripper_cfg.get('open_positions', []),
            'gripper_closed_positions': gripper_cfg.get('closed_positions', []),
            'gripper_command_epsilon': float(gripper_cfg.get('command_epsilon', 0.01)),
            'max_base_velocity': float(base_cfg.get('max_linear_velocity', 0.5)),
            'max_arm_velocity': float(arm_cfg.get('max_joint_velocity', 1.0)),
            'safety_timeout_sec': effective_safety_timeout_sec,
            'workspace_frame_id': workspace_cfg.get(
                'frame_id',
                robot_frames.get('arm_base', 'piper_base_link'),
            ),
            'workspace_x_min': float(workspace_cfg.get('x_min', 0.1)),
            'workspace_x_max': float(workspace_cfg.get('x_max', 0.8)),
            'workspace_y_min': float(workspace_cfg.get('y_min', -0.5)),
            'workspace_y_max': float(workspace_cfg.get('y_max', 0.5)),
            'workspace_z_min': float(workspace_cfg.get('z_min', 0.0)),
            'workspace_z_max': float(workspace_cfg.get('z_max', 1.0)),
            'eef_target_is_delta': bool(action_cfg.get('eef_pose_is_delta', False)),
            'arm_base_frame': robot_frames.get('arm_base', 'piper_base_link'),
            'arm_execution_mode': arm_execution_mode,
            'servo_cartesian_topic': moveit_cfg.get(
                'servo_cartesian_topic', '/servo_node/delta_twist_cmds'
            ),
            'servo_publish_rate_hz': float(moveit_cfg.get('servo_publish_rate_hz', 30.0)),
            'servo_command_horizon_sec': float(effective_servo_horizon_sec),
            'servo_max_linear_velocity': float(
                moveit_cfg.get('servo_max_linear_velocity', 0.10)
            ),
            'servo_max_angular_velocity': float(
                moveit_cfg.get('servo_max_angular_velocity', 0.35)
            ),
            'pause_base_during_servo': bool(
                moveit_cfg.get('pause_base_during_servo', True)
            ),
            'servo_start_service': moveit_cfg.get(
                'servo_start_service', '/servo_node/start_servo'
            ),
            'wait_for_servo_ready': bool(
                moveit_cfg.get('wait_for_servo_ready', True)
            ),
            'servo_ready_timeout_sec': float(
                moveit_cfg.get('servo_ready_timeout_sec', 20.0)
            ),
            'visual_servo_state_topic': str(
                vs_debug_cfg.get('state_topic', '/visual_servo/state')
            ),
            'return_to_ready_after_visual_servo': bool(
                vs_cfg.get('return_to_ready_after_pick', True)
            ),
        }]
    )

    # Visual servo node - only in visual_servo mode
    visual_servo_detection_topic = str(
        vs_cfg.get('detection_topic', detection_cfg.get('detection_topic', '/manipulation/target_detections'))
    )

    remote_detection_client_node = Node(
        package='manipulation_detection',
        executable='remote_detection_client',
        name='remote_detection_client',
        output='screen',
        condition=is_visual_servo_mode,
        parameters=[{
            'image_topic': str(
                detection_cfg.get(
                    'image_topic',
                    robot_topics.get(
                        'camera_rgb',
                        '/piper/wrist_camera/piper_d405/color/image_rect_raw',
                    ),
                )
            ),
            'detection_topic': visual_servo_detection_topic,
            'prompt_topic': str(
                detection_cfg.get('prompt_topic', '/visual_servo/target_prompt')
            ),
            'remote_url': str(
                detection_cfg.get('remote_url', 'http://localhost:30543')
            ),
            'request_rate_hz': float(detection_cfg.get('request_rate_hz', 4.0)),
            'request_timeout_sec': float(detection_cfg.get('request_timeout_sec', 0.30)),
            'retry_attempts': int(detection_cfg.get('retry_attempts', 0)),
            'max_result_staleness_sec': float(
                detection_cfg.get('max_result_staleness_sec', 0.40)
            ),
            'jpeg_quality': int(detection_cfg.get('jpeg_quality', 70)),
            'max_image_long_side_px': int(
                detection_cfg.get('max_image_long_side_px', 640)
            ),
            'default_prompt': str(detection_cfg.get('default_prompt', '')),
            'box_threshold': float(detection_cfg.get('box_threshold', 0.35)),
            'text_threshold': float(detection_cfg.get('text_threshold', 0.25)),
            'min_score': float(detection_cfg.get('min_score', 0.35)),
            'max_detections': int(detection_cfg.get('max_detections', 5)),
            'metrics_log_interval_sec': float(
                detection_cfg.get('metrics_log_interval_sec', 5.0)
            ),
        }],
    )

    visual_servo_node = Node(
        package='manipulation_visual_servo',
        executable='visual_servo_node',
        name='visual_servo_node',
        output='screen',
        condition=is_visual_servo_mode,
        parameters=[{
            'rgb_topic': robot_topics.get('camera_rgb',
                '/piper/wrist_camera/piper_d405/color/image_rect_raw'),
            'camera_info_topic': robot_topics.get('camera_info',
                '/piper/wrist_camera/piper_d405/color/camera_info'),
            'depth_topic': robot_topics.get('camera_depth',
                '/piper/wrist_camera/piper_d405/depth/image_rect_raw'),
            'detection_topic': visual_servo_detection_topic,
            'output_topic': '/manipulation/policy_output',
            'joint_states_topic': joint_states_topic,
            'use_depth': bool(vs_cfg.get('use_depth', False)),
            'control_rate_hz': float(vs_cfg.get('control_rate_hz', 20.0)),
            'output_delta_horizon_sec': float(effective_servo_horizon_sec),
            'target_class': str(vs_cfg.get('target_class', '')),
            'min_detection_confidence': float(
                vs_cfg.get('min_detection_confidence', 0.4)),
            'min_tracking_confidence': float(
                vs_cfg.get('min_tracking_confidence', 0.5)),
            'lost_target_timeout_sec': float(
                vs_cfg.get('lost_target_timeout_sec', 0.3)),
            'acquire_timeout_sec': float(
                vs_cfg.get('acquire_timeout_sec', 5.0)),
            'image_center_tolerance_px': float(
                vs_cfg.get('image_center_tolerance_px', 8.0)),
            'grasp_standoff_m': float(vs_cfg.get('grasp_standoff_m', 0.115)),
            'grasp_depth_tolerance_m': float(
                vs_cfg.get('grasp_depth_tolerance_m', 0.015)),
            'depth_sample_anchor_x': float(
                vs_cfg.get('depth_sample_anchor_x', 0.50)),
            'depth_sample_anchor_y': float(
                vs_cfg.get('depth_sample_anchor_y', 0.68)),
            'depth_roi_half_size_px': int(vs_cfg.get('depth_roi_half_size_px', 8)),
            'min_valid_depth_pixels': int(
                vs_cfg.get('min_valid_depth_pixels', 12)),
            'depth_sample_max_iqr_m': float(
                vs_cfg.get('depth_sample_max_iqr_m', 0.015)),
            'depth_stale_timeout_sec': float(
                vs_cfg.get('depth_stale_timeout_sec', 0.25)),
            'centering_stable_cycles': int(
                vs_cfg.get('centering_stable_cycles', 3)),
            'close_depth_stable_frames': int(
                vs_cfg.get('close_depth_stable_frames', 3)),
            'grasp_settle_sec': float(vs_cfg.get('grasp_settle_sec', 0.75)),
            'lift_distance_m': float(vs_cfg.get('lift_distance_m', 0.08)),
            'max_approach_distance_m': float(
                vs_cfg.get('max_approach_distance_m', 0.50)),
            'approach_stall_window_sec': float(
                vs_cfg.get('approach_stall_window_sec', 1.0)),
            'approach_min_progress_m': float(
                vs_cfg.get('approach_min_progress_m', 0.01)),
            'blind_approach_depth_threshold_m': float(
                vs_cfg.get('blind_approach_depth_threshold_m', 0.30)),
            'blind_approach_velocity_fraction': float(
                vs_cfg.get('blind_approach_velocity_fraction', 0.5)),
            'blind_approach_max_distance_m': float(
                vs_cfg.get('blind_approach_max_distance_m', 0.20)),
            'blind_approach_after_standoff_m': float(
                vs_cfg.get('blind_approach_after_standoff_m', 0.03)),
            'blind_push_timeout_sec': float(
                vs_cfg.get('blind_push_timeout_sec', 7.2)),
            'blind_push_close_tolerance_m': float(
                vs_cfg.get('blind_push_close_tolerance_m', 0.006)),
            'blind_push_offset_x': float(
                vs_cfg.get('blind_push_offset_x', 0.0)),
            'blind_push_offset_y': float(
                vs_cfg.get('blind_push_offset_y', 0.0)),
            'open_gripper_settle_sec': float(
                vs_cfg.get('open_gripper_settle_sec', 3.0)),
            'gripper_cmd_action': robot_actions.get(
                'gripper_command', '/piper_gripper_controller/gripper_cmd'
            ),
            'gripper_joint_name': gripper_cfg.get('joint_name', 'piper_joint7'),
            'gripper_joint_names': gripper_cfg.get('joint_names', []),
            'gripper_open_position': float(gripper_cfg.get('open_position', 0.065)),
            'gripper_closed_position': float(gripper_cfg.get('closed_position', 0.0)),
            'gripper_open_positions': gripper_cfg.get('open_positions', []),
            'gripper_open_position_tolerance': float(
                vs_cfg.get('gripper_open_position_tolerance', 0.02)),
            'reference_frame': str(vs_cfg.get(
                'reference_frame',
                robot_frames.get('arm_base', 'piper_base_link'))),
            'camera_optical_frame': str(vs_cfg.get(
                'camera_optical_frame',
                robot_frames.get('camera_optical', 'piper_camera_optical_frame'))),
            'ee_frame': str(vs_cfg.get(
                'ee_frame',
                robot_frames.get('ee_link', 'piper_tcp'))),
            'arm_base_frame': str(vs_cfg.get(
                'arm_base_frame',
                robot_frames.get('arm_base', 'piper_base_link'))),
            'tracker_type': str(vs_cfg.get('tracker_type', 'mil')),
            'klt_max_features': int(vs_cfg.get('klt_max_features', 200)),
            'klt_quality_level': float(vs_cfg.get('klt_quality_level', 0.01)),
            'klt_min_distance': float(vs_cfg.get('klt_min_distance', 5.0)),
            'klt_window_size': int(vs_cfg.get('klt_window_size', 10)),
            'klt_pyramid_levels': int(vs_cfg.get('klt_pyramid_levels', 3)),
            'control.lambda_xy': float(vs_control_cfg.get('lambda_xy', 0.3)),
            'control.lambda_z': float(vs_control_cfg.get('lambda_z', 0.1)),
            'control.lambda_rz': float(vs_control_cfg.get('lambda_rz', 0.1)),
            'control.max_linear_velocity': float(
                vs_control_cfg.get('max_linear_velocity', 0.08)),
            'control.max_angular_velocity': float(
                vs_control_cfg.get('max_angular_velocity', 0.30)),
            'control.ramp_up_steps': int(
                vs_control_cfg.get('ramp_up_steps', 5)),
            'debug.publish_overlay': bool(
                vs_debug_cfg.get('publish_overlay', True)),
            'debug.overlay_topic': str(
                vs_debug_cfg.get('overlay_topic', '/visual_servo/debug_image')),
            'debug.publish_state': bool(
                vs_debug_cfg.get('publish_state', True)),
            'debug.state_topic': str(
                vs_debug_cfg.get('state_topic', '/visual_servo/state')),
        }]
    )

    return LaunchDescription([
        # Arguments
        use_remote_policy_arg,
        remote_url_arg,
        camera_topic_arg,
        arm_execution_mode_arg,
        control_mode_arg,

        # Nodes (always launched)
        perception_node,
        *bridge_virtual_nodes,
        adapter_node,

        # VLA mode nodes (conditional)
        policy_node_local,
        policy_node_remote,

        # Visual servo mode nodes (conditional)
        remote_detection_client_node,
        visual_servo_node,
    ])
