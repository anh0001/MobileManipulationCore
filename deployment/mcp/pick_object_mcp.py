#!/usr/bin/env python3
# Copyright 2026 MobileManipulationCore Contributors
# Licensed under the Apache License, Version 2.0.
"""MCP server exposing the PickObject ROS 2 action as a single LLM-callable tool.

Lets any MCP client (Claude Code/Desktop, other LLM agents) ask the robot to
pick an object by name, e.g. pick_object("bread"). Wraps the /pick_object action
served by manipulation_policy/pick_orchestrator.

Run inside a sourced ROS 2 environment (the grasp pipeline must be running):
    source /opt/ros/humble/setup.bash
    source <ws>/install/setup.bash
    python3 deployment/mcp/pick_object_mcp.py

Register in an MCP client (example, Claude Code .mcp.json):
    {
      "mcpServers": {
        "robot-pick": {
          "command": "bash",
          "args": ["-lc",
            "source /opt/ros/humble/setup.bash && source ~/codes/MobileManipulationCore/install/setup.bash && python3 ~/codes/MobileManipulationCore/deployment/mcp/pick_object_mcp.py"]
        }
      }
    }

Requires: pip install "mcp[cli]"  (the official Python MCP SDK).
"""
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from manipulation_msgs.action import PickObject

from mcp.server.fastmcp import FastMCP

mcp = FastMCP("robot-pick")

_node = None
_client = None
_lock = threading.Lock()


def _ensure_ros():
    """Lazily init rclpy + a spinning node with the PickObject action client."""
    global _node, _client
    if _node is not None:
        return
    if not rclpy.ok():
        rclpy.init()
    _node = Node("pick_object_mcp")
    _client = ActionClient(_node, PickObject, "/pick_object")
    threading.Thread(target=lambda: rclpy.spin(_node), daemon=True).start()


@mcp.tool()
def pick_object(object: str, timeout_sec: float = 70.0) -> dict:
    """Pick a single object by name with the robot arm.

    Args:
        object: open-vocabulary label of the thing to grasp, e.g. "bread",
            "toy banana", "red apple". For toy fruits prefix with "toy".
        timeout_sec: max seconds to allow for the whole attempt.

    Returns a dict: success (reached the grasp + lift), object_held (jaw blocked
    on something — heuristic, no force sensor), gripper_width (m), message.
    """
    with _lock:
        _ensure_ros()
        if not _client.wait_for_server(timeout_sec=5.0):
            return {"success": False, "object_held": False,
                    "message": "/pick_object action server not available "
                               "(is the grasp pipeline running?)"}
        goal = PickObject.Goal()
        goal.object = object
        goal.timeout_sec = float(timeout_sec)

        send = _client.send_goal_async(goal)
        while not send.done():
            time.sleep(0.05)
        gh = send.result()
        if gh is None or not gh.accepted:
            return {"success": False, "object_held": False, "message": "goal rejected"}
        res_future = gh.get_result_async()
        deadline = time.time() + float(timeout_sec) + 30.0
        while not res_future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not res_future.done():
            return {"success": False, "object_held": False,
                    "message": "timed out waiting for action result"}
        res = res_future.result().result
        return {
            "success": bool(res.success),
            "object_held": bool(res.object_held),
            "gripper_width": round(float(res.gripper_width), 4),
            "message": str(res.message),
        }


if __name__ == "__main__":
    mcp.run()
