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

import argparse
import sys
import threading
import time
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
  clear       Alias for stop
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
        self.topic = topic
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
    argv = list(args) if args is not None else sys.argv[1:]
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        '--topic',
        default='/manipulation/task_prompt',
        help='Topic to publish task prompts to',
    )
    parser.add_argument(
        '--wait-for-subscriber-sec',
        type=float,
        default=2.0,
        help='Wait time for policy subscriber before entering prompt loop',
    )
    known_args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)

    topic = known_args.topic

    node = TaskPromptNode(topic)

    # Run rclpy spin in background so publisher works without blocking input
    t = threading.Thread(target=spin_thread, args=(node,), daemon=True)
    t.start()

    print(HELP_TEXT)
    wait_sec = max(0.0, float(known_args.wait_for_subscriber_sec))
    if wait_sec > 0.0:
        deadline = time.monotonic() + wait_sec
        while time.monotonic() < deadline and node.pub.get_subscription_count() == 0:
            time.sleep(0.05)

    if node.pub.get_subscription_count() == 0:
        print(f'Publishing to: {topic} (no subscribers detected yet)')
    else:
        print(f'Publishing to: {topic} ({node.pub.get_subscription_count()} subscriber(s))')
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
                print(f'Topic: {node.topic}')
                print(f'Subscribers: {node.pub.get_subscription_count()}')
            elif lower in ('stop', 'clear'):
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
