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
Launch file to spawn robot in Gazebo for simulation testing.

This launch file starts Gazebo with a specified world and spawns the robot model.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('manipulation_bringup')  # Adjust if needed

    # Declare arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_world',
        description='World name (without .sdf extension)'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo GUI'
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode (no GUI, no rendering)'
    )

    # Get launch configurations
    world_name = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')

    # Build world file path
    # Note: Update this path based on where you place this launch file
    world_path = PathJoinSubstitution([
        FindPackageShare('manipulation_bringup'),
        'worlds',
        [world_name, '.sdf']
    ])

    # Alternative: If worlds are in the sim/ directory
    # world_path = os.path.join(os.path.dirname(__file__), '..', 'worlds', f'{world_name}.sdf')

    # Start Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_path],
        output='screen',
    )

    # Start Gazebo client (GUI) conditionally
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(gui)
    )

    # TODO: Spawn robot model
    # This requires a URDF file describing your robot
    # spawn_robot = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=[
    #         '-entity', 'mobile_manipulator',
    #         '-file', 'path/to/robot.urdf',  # or use -topic /robot_description
    #         '-x', '0', '-y', '0', '-z', '0.1'
    #     ],
    #     output='screen'
    # )

    # TODO: Load robot description to parameter server
    # robot_description = ...

    # TODO: Start robot state publisher
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     parameters=[{'robot_description': robot_description}]
    # )

    return LaunchDescription([
        # Arguments
        world_arg,
        gui_arg,
        headless_arg,

        # Gazebo
        gzserver,
        gzclient,

        # Robot spawning (uncomment when URDF is ready)
        # spawn_robot,
        # robot_state_publisher,
    ])
