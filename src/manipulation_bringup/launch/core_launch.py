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

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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

    robot_cfg = load_yaml(resolve_config_path('robot_params.yaml'))
    policy_cfg_full = load_yaml(resolve_config_path('policy_params.yaml'))

    robot_topics = robot_cfg.get('robot', {}).get('topics', {})
    robot_frames = robot_cfg.get('robot', {}).get('frames', {})
    robot_actions = robot_cfg.get('robot', {}).get('actions', {})
    base_cfg = robot_cfg.get('base', {})
    arm_cfg = robot_cfg.get('arm', {})
    workspace_cfg = robot_cfg.get('workspace', {})

    policy_cfg = policy_cfg_full.get('policy', {})
    action_cfg = policy_cfg_full.get('action', {})
    remote_cfg = policy_cfg_full.get('remote', {})
    safety_cfg = policy_cfg_full.get('safety', {})
    observation_cfg = policy_cfg_full.get('observation', {})

    image_size = policy_cfg.get('image_size', [224, 224])
    image_width = image_size[0] if isinstance(image_size, (list, tuple)) and len(image_size) > 0 else 224
    image_height = image_size[1] if isinstance(image_size, (list, tuple)) and len(image_size) > 1 else 224
    observation_topic = observation_cfg.get('topic', '/manipulation/observation')

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

    # Get launch configurations
    use_remote_policy = LaunchConfiguration('use_remote_policy')
    remote_url = LaunchConfiguration('remote_url')
    camera_topic = LaunchConfiguration('camera_topic')
    joint_states_topic = robot_topics.get('joint_states', '/joint_states')

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

    # Policy node (local)
    policy_node_local = Node(
        package='manipulation_policy',
        executable='policy_node',
        name='policy_node',
        output='screen',
        condition=UnlessCondition(use_remote_policy),
        parameters=[{
            'model_name': policy_cfg.get('model_name', 'openvla-7b'),
            'model_path': policy_cfg.get('model_path', ''),
            'use_remote': False,
            'inference_rate': float(policy_cfg.get('inference_rate', 10.0)),
            'device': policy_cfg.get('device', 'cuda'),
            'use_fp16': bool(policy_cfg.get('use_fp16', True)),
            'remote_timeout_sec': float(remote_cfg.get('timeout', 1.0)),
            'remote_retry_attempts': int(remote_cfg.get('retry_attempts', 3)),
            'remote_fallback_on_failure': bool(remote_cfg.get('fallback_on_failure', True)),
            'camera_topic': camera_topic,
            'camera_frame': robot_frames.get('camera_link', ''),
            'joint_states_topic': joint_states_topic,
            'use_observation': True,
            'observation_topic': observation_topic,
        }]
    )

    # Policy node (remote client)
    policy_node_remote = Node(
        package='manipulation_policy',
        executable='policy_node',
        name='policy_node',
        output='screen',
        condition=IfCondition(use_remote_policy),
        parameters=[{
            'use_remote': True,
            'remote_url': remote_url,
            'inference_rate': float(policy_cfg.get('inference_rate', 10.0)),
            'device': policy_cfg.get('device', 'cuda'),
            'use_fp16': bool(policy_cfg.get('use_fp16', True)),
            'remote_timeout_sec': float(remote_cfg.get('timeout', 1.0)),
            'remote_retry_attempts': int(remote_cfg.get('retry_attempts', 3)),
            'remote_fallback_on_failure': bool(remote_cfg.get('fallback_on_failure', True)),
            'camera_topic': camera_topic,
            'camera_frame': robot_frames.get('camera_link', ''),
            'joint_states_topic': joint_states_topic,
            'use_observation': True,
            'observation_topic': observation_topic,
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
            'ee_frame': robot_frames.get('ee_link', 'piper_gripper'),
            'joint_states_topic': robot_topics.get('joint_states', '/joint_states'),
            'navigate_to_pose_action': robot_actions.get('navigate_to_pose', '/navigate_to_pose'),
            'follow_joint_trajectory_action': robot_actions.get(
                'follow_joint_trajectory', '/piper_arm_controller/follow_joint_trajectory'
            ),
            'max_base_velocity': float(base_cfg.get('max_linear_velocity', 0.5)),
            'max_arm_velocity': float(arm_cfg.get('max_joint_velocity', 1.0)),
            'safety_timeout_sec': float(safety_cfg.get('timeout_sec', 2.0)),
            'workspace_x_min': float(workspace_cfg.get('x_min', 0.1)),
            'workspace_x_max': float(workspace_cfg.get('x_max', 0.8)),
            'workspace_y_min': float(workspace_cfg.get('y_min', -0.5)),
            'workspace_y_max': float(workspace_cfg.get('y_max', 0.5)),
            'workspace_z_min': float(workspace_cfg.get('z_min', 0.0)),
            'workspace_z_max': float(workspace_cfg.get('z_max', 1.0)),
            'eef_target_is_delta': bool(action_cfg.get('eef_pose_is_delta', False)),
        }]
    )

    return LaunchDescription([
        # Arguments
        use_remote_policy_arg,
        remote_url_arg,
        camera_topic_arg,

        # Nodes
        perception_node,
        policy_node_local,
        policy_node_remote,
        adapter_node,
    ])
