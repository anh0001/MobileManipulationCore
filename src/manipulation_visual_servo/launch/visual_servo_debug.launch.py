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
Debug launch file for isolated hybrid pick visual servo node testing.

This launches only the visual_servo_node with configurable parameters,
without the adapter or perception nodes. Useful for bench testing with
bags or simulated detection feeds.

The hybrid pick pipeline requires MoveIt Servo to be available for the
FINAL_SERVO phase. The adapter must be configured with both move_group
and moveit_servo support since this node sends per-message mode overrides.
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

    servo_ctrl = vs_cfg.get('servo_control', {})
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
                # Topics
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'detection_topic': LaunchConfiguration('detection_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'joint_states_topic': str(vs_cfg.get(
                    'joint_states_topic', '/joint_states')),
                # Frames
                'reference_frame': LaunchConfiguration('reference_frame'),
                'camera_optical_frame': LaunchConfiguration('camera_optical_frame'),
                'ee_frame': str(vs_cfg.get('ee_frame', 'piper_link6')),
                'arm_base_frame': str(vs_cfg.get('arm_base_frame', 'piper_base_link')),
                # General
                'control_rate_hz': float(vs_cfg.get('control_rate_hz', 20.0)),
                'target_class': str(vs_cfg.get('target_class', '')),
                'min_detection_confidence': float(
                    vs_cfg.get('min_detection_confidence', 0.4)),
                # Timeouts
                'lost_target_timeout_sec': float(
                    vs_cfg.get('lost_target_timeout_sec', 2.0)),
                'acquire_timeout_sec': float(
                    vs_cfg.get('acquire_timeout_sec', 5.0)),
                'estimate_timeout_sec': float(
                    vs_cfg.get('estimate_timeout_sec', 3.0)),
                'pregrasp_timeout_sec': float(
                    vs_cfg.get('pregrasp_timeout_sec', 15.0)),
                'final_servo_timeout_sec': float(
                    vs_cfg.get('final_servo_timeout_sec', 10.0)),
                'verify_timeout_sec': float(
                    vs_cfg.get('verify_timeout_sec', 2.0)),
                'lift_timeout_sec': float(
                    vs_cfg.get('lift_timeout_sec', 10.0)),
                # Depth sampling
                'depth_roi_body_top_frac': float(
                    vs_cfg.get('depth_roi_body_top_frac', 0.30)),
                'depth_roi_body_bottom_frac': float(
                    vs_cfg.get('depth_roi_body_bottom_frac', 0.90)),
                'depth_roi_body_left_frac': float(
                    vs_cfg.get('depth_roi_body_left_frac', 0.20)),
                'depth_roi_body_right_frac': float(
                    vs_cfg.get('depth_roi_body_right_frac', 0.80)),
                'min_valid_depth_pixels': int(
                    vs_cfg.get('min_valid_depth_pixels', 10)),
                'depth_sample_max_iqr_m': float(
                    vs_cfg.get('depth_sample_max_iqr_m', 0.03)),
                'depth_stale_timeout_sec': float(
                    vs_cfg.get('depth_stale_timeout_sec', 1.0)),
                # Hybrid pick
                'pregrasp_offset_m': float(
                    vs_cfg.get('pregrasp_offset_m', 0.08)),
                'final_servo_distance_m': float(
                    vs_cfg.get('final_servo_distance_m', 0.04)),
                'grasp_settle_sec': float(
                    vs_cfg.get('grasp_settle_sec', 1.0)),
                'lift_distance_m': float(
                    vs_cfg.get('lift_distance_m', 0.08)),
                'retreat_distance_m': float(
                    vs_cfg.get('retreat_distance_m', 0.05)),
                # Grasp orientation
                'bottle_grasp_orientation_x': float(
                    vs_cfg.get('bottle_grasp_orientation_x', 1.0)),
                'bottle_grasp_orientation_y': float(
                    vs_cfg.get('bottle_grasp_orientation_y', 0.0)),
                'bottle_grasp_orientation_z': float(
                    vs_cfg.get('bottle_grasp_orientation_z', 0.0)),
                'bottle_grasp_orientation_w': float(
                    vs_cfg.get('bottle_grasp_orientation_w', 0.0)),
                # Convergence
                'final_position_tolerance_m': float(
                    vs_cfg.get('final_position_tolerance_m', 0.008)),
                'final_image_tolerance_px': float(
                    vs_cfg.get('final_image_tolerance_px', 12.0)),
                'final_convergence_cycles': int(
                    vs_cfg.get('final_convergence_cycles', 3)),
                # Gripper
                'open_gripper_command': float(
                    vs_cfg.get('open_gripper_command', 1.0)),
                'close_gripper_command': float(
                    vs_cfg.get('close_gripper_command', 0.0)),
                'grasp_success_min_width': float(
                    vs_cfg.get('grasp_success_min_width', 0.003)),
                'gripper_closed_position': float(
                    vs_cfg.get('gripper_closed_position', 0.0)),
                'gripper_joint_name': str(
                    vs_cfg.get('gripper_joint_name', 'piper_joint7')),
                # Servo control
                'servo_control.lambda_xy': float(
                    servo_ctrl.get('lambda_xy', 2.0)),
                'servo_control.lambda_z': float(
                    servo_ctrl.get('lambda_z', 2.0)),
                'servo_control.max_linear_velocity': float(
                    servo_ctrl.get('max_linear_velocity', 0.05)),
                'servo_control.max_angular_velocity': float(
                    servo_ctrl.get('max_angular_velocity', 0.20)),
                'servo_control.ramp_up_steps': int(
                    servo_ctrl.get('ramp_up_steps', 3)),
                # Debug
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
