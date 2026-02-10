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
Interactive CLI for sending task prompts to the policy node.

Run this in a terminal alongside the robot stack:
    ros2 run manipulation_policy task_prompt_cli

Then type prompts like:
    > pick up the bottle
    > place it on the table
    > open the drawer
    > stop   (or press Ctrl+C to exit)
"""

import sys
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


HELP_TEXT = """
Task Prompt CLI - Send language instructions to the robot
----------------------------------------------------------
Type a task description and press Enter to send it.
The robot will execute the task using the VLA policy.

Commands:
  stop        Clear the current task (robot stops acting)
  status      Show the last task sent
  help        Show this help message
  quit / exit Exit the CLI

Examples:
  pick up the bottle
  place the cup on the table
  open the drawer
  move to the charging station
----------------------------------------------------------
"""


class TaskPromptNode(Node):
    def __init__(self, topic: str):
        super().__init__('task_prompt_cli')
        self.pub = self.create_publisher(String, topic, 10)
        self.last_task = ''

    def send(self, task: str):
        msg = String()
        msg.data = task
        self.pub.publish(msg)
        self.last_task = task


def spin_thread(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)

    topic = '/manipulation/task_prompt'
    # Allow override via CLI: task_prompt_cli --ros-args -p topic:=/my/topic
    # (rclpy handles --ros-args automatically; topic param not declared here
    #  since it's a simple publisher — use the default or change the source)

    node = TaskPromptNode(topic)

    # Run rclpy spin in background so publisher works without blocking input
    t = threading.Thread(target=spin_thread, args=(node,), daemon=True)
    t.start()

    print(HELP_TEXT)
    print(f'Publishing to: {topic}')
    print()

    try:
        while rclpy.ok():
            try:
                raw = input('> ').strip()
            except EOFError:
                break

            if not raw:
                continue

            lower = raw.lower()

            if lower in ('quit', 'exit'):
                break
            elif lower == 'help':
                print(HELP_TEXT)
            elif lower == 'status':
                if node.last_task:
                    print(f'Current task: "{node.last_task}"')
                else:
                    print('No task sent yet.')
            elif lower == 'stop':
                node.send('')
                print('Task cleared. Robot will stop acting on previous prompt.')
            else:
                node.send(raw)
                print(f'Sent: "{raw}"')

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('\nExiting task prompt CLI.')


if __name__ == '__main__':
    main()
