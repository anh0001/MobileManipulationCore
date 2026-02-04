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

"""Launch Ranger Garden Assistant base stack with the manipulation stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_base_arg = DeclareLaunchArgument(
        'launch_base',
        default_value='true',
        description='Launch robofi_bringup ranger_complete_bringup.launch.py',
    )

    use_remote_policy_arg = DeclareLaunchArgument(
        'use_remote_policy',
        default_value='false',
        description='Use remote policy server for inference',
    )

    remote_url_arg = DeclareLaunchArgument(
        'remote_url',
        default_value='http://localhost:5000',
        description='URL of remote policy server',
    )

    launch_base = LaunchConfiguration('launch_base')
    use_remote_policy = LaunchConfiguration('use_remote_policy')
    remote_url = LaunchConfiguration('remote_url')

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robofi_bringup'),
                'launch',
                'ranger_complete_bringup.launch.py',
            ])
        ]),
        condition=IfCondition(launch_base),
    )

    manipulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('manipulation_bringup'),
                'launch',
                'core_launch.py',
            ])
        ]),
        launch_arguments={
            'use_remote_policy': use_remote_policy,
            'remote_url': remote_url,
        }.items(),
    )

    return LaunchDescription([
        launch_base_arg,
        use_remote_policy_arg,
        remote_url_arg,
        base_launch,
        manipulation_launch,
    ])
