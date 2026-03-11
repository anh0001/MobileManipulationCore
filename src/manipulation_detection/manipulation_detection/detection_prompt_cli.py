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

"""Interactive CLI for runtime updates of visual-servo detection prompt."""

import argparse
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


HELP_TEXT = """
Detection Prompt CLI - Grounding DINO prompt updates
----------------------------------------------------
Type a detector prompt and press Enter to send it.
Examples:
  red bottle
  cardboard box
  screwdriver

Commands:
  clear       Clear prompt (detector requests pause)
  status      Show current prompt and subscriber count
  help        Show this help
  quit / exit Exit
----------------------------------------------------
"""


class DetectionPromptNode(Node):
    def __init__(self, topic: str):
        super().__init__("detection_prompt_cli")
        self.topic = topic
        self.publisher = self.create_publisher(String, topic, 10)
        self.last_prompt = ""

    def send(self, prompt: str):
        msg = String()
        msg.data = prompt
        self.publisher.publish(msg)
        self.last_prompt = prompt


def _spin(node: Node):
    rclpy.spin(node)


def main(args=None):
    argv = list(args) if args is not None else sys.argv[1:]
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--topic",
        default="/visual_servo/target_prompt",
        help="Topic to publish detector prompt to",
    )
    parser.add_argument(
        "--wait-for-subscriber-sec",
        type=float,
        default=2.0,
        help="Wait this long for subscribers before starting prompt loop",
    )
    known_args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = DetectionPromptNode(known_args.topic)
    thread = threading.Thread(target=_spin, args=(node,), daemon=True)
    thread.start()

    print(HELP_TEXT)
    wait_sec = max(0.0, float(known_args.wait_for_subscriber_sec))
    if wait_sec > 0.0:
        deadline = time.monotonic() + wait_sec
        while time.monotonic() < deadline and node.publisher.get_subscription_count() == 0:
            time.sleep(0.05)

    print(
        f"Publishing to: {node.topic} "
        f"(subscribers={node.publisher.get_subscription_count()})"
    )

    try:
        while rclpy.ok():
            try:
                raw = input("> ").strip()
            except EOFError:
                break

            if not raw:
                continue

            lowered = raw.lower()
            if lowered in ("quit", "exit"):
                break
            if lowered == "help":
                print(HELP_TEXT)
                continue
            if lowered == "status":
                if node.last_prompt:
                    print(f"Current prompt: '{node.last_prompt}'")
                else:
                    print("Current prompt: <empty>")
                print(f"Topic: {node.topic}")
                print(f"Subscribers: {node.publisher.get_subscription_count()}")
                continue
            if lowered == "clear":
                node.send("")
                print("Prompt cleared.")
                continue

            node.send(raw)
            print(f"Sent prompt: '{raw}'")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("\nExiting detection prompt CLI.")


if __name__ == "__main__":
    main()
