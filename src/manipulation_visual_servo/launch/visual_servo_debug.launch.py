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
Debug launch file for isolated visual servo node testing.

This launches only the visual_servo_node with configurable parameters,
without the adapter or perception nodes. Useful for bench testing with
bags or simulated detection feeds.
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('manipulation_visual_servo')
    default_config = os.path.join(pkg_share, 'config', 'visual_servo_params.yaml')

    # Load defaults from config file
    vs_cfg = {}
    try:
        with open(default_config, 'r', encoding='utf-8') as f:
            full_cfg = yaml.safe_load(f) or {}
            vs_cfg = full_cfg.get('visual_servo', {})
    except FileNotFoundError:
        pass

    control_cfg = vs_cfg.get('control', {})
    debug_cfg = vs_cfg.get('debug', {})

    return LaunchDescription([
        DeclareLaunchArgument(
            'rgb_topic',
            default_value=vs_cfg.get(
                'rgb_topic',
                '/piper/wrist_camera/piper_d405/color/image_rect_raw'),
            description='Raw RGB image topic'),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value=vs_cfg.get(
                'camera_info_topic',
                '/piper/wrist_camera/piper_d405/color/camera_info'),
            description='Camera info topic'),
        DeclareLaunchArgument(
            'depth_topic',
            default_value=vs_cfg.get(
                'depth_topic',
                '/piper/wrist_camera/piper_d405/depth/image_rect_raw'),
            description='Depth image topic'),
        DeclareLaunchArgument(
            'detection_topic',
            default_value=vs_cfg.get(
                'detection_topic',
                '/manipulation/target_detections'),
            description='Detection2DArray topic'),
        DeclareLaunchArgument(
            'output_topic',
            default_value=vs_cfg.get(
                'output_topic',
                '/manipulation/policy_output'),
            description='PolicyOutput output topic'),
        DeclareLaunchArgument(
            'reference_frame',
            default_value=vs_cfg.get('reference_frame', 'piper_base_link'),
            description='Reference frame for output poses'),
        DeclareLaunchArgument(
            'camera_optical_frame',
            default_value='piper_camera_optical_frame',
            description='Camera optical frame for TF'),
        DeclareLaunchArgument(
            'control_rate_hz',
            default_value=str(vs_cfg.get('control_rate_hz', 20.0)),
            description='Control loop rate in Hz'),

        Node(
            package='manipulation_visual_servo',
            executable='visual_servo_node',
            name='visual_servo_node',
            output='screen',
            parameters=[{
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'detection_topic': LaunchConfiguration('detection_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'reference_frame': LaunchConfiguration('reference_frame'),
                'camera_optical_frame': LaunchConfiguration('camera_optical_frame'),
                'control_rate_hz': float(vs_cfg.get('control_rate_hz', 20.0)),
                'output_delta_horizon_sec': float(vs_cfg.get('output_delta_horizon_sec', 0.0)),
                'use_depth': bool(vs_cfg.get('use_depth', False)),
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
                'open_gripper_command': float(
                    vs_cfg.get('open_gripper_command', 1.0)),
                'close_gripper_command': float(
                    vs_cfg.get('close_gripper_command', 0.0)),
                'tracker_type': str(vs_cfg.get('tracker_type', 'klt')),
                'klt_max_features': int(vs_cfg.get('klt_max_features', 200)),
                'klt_quality_level': float(vs_cfg.get('klt_quality_level', 0.01)),
                'klt_min_distance': float(vs_cfg.get('klt_min_distance', 5.0)),
                'klt_window_size': int(vs_cfg.get('klt_window_size', 10)),
                'klt_pyramid_levels': int(vs_cfg.get('klt_pyramid_levels', 3)),
                'control.lambda_xy': float(control_cfg.get('lambda_xy', 0.3)),
                'control.lambda_z': float(control_cfg.get('lambda_z', 0.1)),
                'control.lambda_rz': float(control_cfg.get('lambda_rz', 0.1)),
                'control.max_linear_velocity': float(
                    control_cfg.get('max_linear_velocity', 0.08)),
                'control.max_angular_velocity': float(
                    control_cfg.get('max_angular_velocity', 0.30)),
                'control.ramp_up_steps': int(control_cfg.get('ramp_up_steps', 5)),
                'debug.publish_overlay': bool(
                    debug_cfg.get('publish_overlay', True)),
                'debug.overlay_topic': str(
                    debug_cfg.get('overlay_topic', '/visual_servo/debug_image')),
                'debug.publish_state': bool(
                    debug_cfg.get('publish_state', True)),
                'debug.state_topic': str(
                    debug_cfg.get('state_topic', '/visual_servo/state')),
            }],
        ),
    ])
