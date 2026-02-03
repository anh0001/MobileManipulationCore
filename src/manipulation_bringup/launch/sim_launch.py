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
Launch file for simulation testing.

This launch file starts the manipulation stack in simulation mode
for safe testing without hardware.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_world',
        description='Gazebo world to load'
    )

    # Get configurations
    world = LaunchConfiguration('world')

    # TODO: Include Gazebo launch file
    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('manipulation_bringup'),
    #             'launch',
    #             'gazebo_launch.py'
    #         ])
    #     ]),
    #     launch_arguments={'world': world}.items()
    # )

    # Include core manipulation stack
    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('manipulation_bringup'),
                'launch',
                'core_launch.py'
            ])
        ]),
        launch_arguments={
            'use_remote_policy': 'false',
            'camera_topic': '/camera/color/image_raw'
        }.items()
    )

    return LaunchDescription([
        # Arguments
        world_arg,

        # Launches
        # gazebo_launch,  # TODO: Uncomment when Gazebo setup is ready
        core_launch,
    ])
