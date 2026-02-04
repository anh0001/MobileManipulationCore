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

"""Launch file for the remote policy server."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    host_arg = DeclareLaunchArgument(
        "host",
        default_value="0.0.0.0",
        description="Host interface for the policy server",
    )
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="5000",
        description="Port for the policy server",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level for the policy server",
    )

    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    log_level = LaunchConfiguration("log_level")

    server = ExecuteProcess(
        cmd=[
            "python3",
            "-m",
            "manipulation_policy.policy_server",
            "--host",
            host,
            "--port",
            port,
            "--log-level",
            log_level,
        ],
        output="screen",
    )

    return LaunchDescription([
        host_arg,
        port_arg,
        log_level_arg,
        server,
    ])
