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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    use_remote_policy_arg = DeclareLaunchArgument(
        'use_remote_policy',
        default_value='false',
        description='Use remote policy server for inference'
    )

    remote_url_arg = DeclareLaunchArgument(
        'remote_url',
        default_value='http://localhost:5000',
        description='URL of remote policy server'
    )

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/color/image_raw',
        description='Camera topic for visual input'
    )

    # Get launch configurations
    use_remote_policy = LaunchConfiguration('use_remote_policy')
    remote_url = LaunchConfiguration('remote_url')
    camera_topic = LaunchConfiguration('camera_topic')

    # Perception node
    perception_node = Node(
        package='manipulation_perception',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[{
            'camera_topic': camera_topic,
            'depth_topic': '/camera/depth/image_raw',
            'joint_states_topic': '/joint_states'
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
            'model_name': 'openvla-7b',
            'use_remote': False,
            'inference_rate': 10.0
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
            'inference_rate': 10.0
        }]
    )

    # Adapter node
    adapter_node = Node(
        package='manipulation_adapter',
        executable='adapter_node',
        name='adapter_node',
        output='screen',
        parameters=[{
            'base_frame': 'base_link',
            'ee_frame': 'piper_gripper',
            'max_base_velocity': 0.5,
            'max_arm_velocity': 1.0,
            'safety_timeout_sec': 2.0
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
