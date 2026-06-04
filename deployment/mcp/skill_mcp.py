#!/usr/bin/env python3
# Copyright 2026 MobileManipulationCore Contributors
# Licensed under the Apache License, Version 2.0.
"""MCP server exposing every registered robot skill as an LLM-callable tool.

Reads the skill registry (manipulation_policy.skills) and generates one MCP tool
per skill, with the skill's declared params as typed tool arguments and its
description as the tool docstring. Each tool sends an ExecuteSkill goal to the
``/execute_skill`` action served by manipulation_policy/skill_server.

Adding a new skill needs NO change here: write the skill, register it, and it
shows up as a new MCP tool the next time this server starts.

Run inside a sourced ROS 2 environment (the robot pipeline must be running):
    source /opt/ros/humble/setup.bash
    source <ws>/install/setup.bash
    python3 deployment/mcp/skill_mcp.py

Register in an MCP client (example, Claude Code .mcp.json):
    {
      "mcpServers": {
        "robot-skills": {
          "command": "bash",
          "args": ["-lc",
            "source /opt/ros/humble/setup.bash && source ~/codes/MobileManipulationCore/install/setup.bash && python3 ~/codes/MobileManipulationCore/deployment/mcp/skill_mcp.py"]
        }
      }
    }

Requires: pip install "mcp[cli]"  (the official Python MCP SDK).
"""
import inspect
import json
import threading
import time
from typing import Any, Dict

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from manipulation_msgs.action import ExecuteSkill
from manipulation_policy.skills import all_skills

from mcp.server.fastmcp import FastMCP

mcp = FastMCP("robot-skills")

_node = None
_client = None
_lock = threading.Lock()


def _ensure_ros():
    """Lazily init rclpy + a spinning node with the ExecuteSkill action client."""
    global _node, _client
    if _node is not None:
        return
    if not rclpy.ok():
        rclpy.init()
    _node = Node("skill_mcp")
    _client = ActionClient(_node, ExecuteSkill, "/execute_skill")
    threading.Thread(target=lambda: rclpy.spin(_node), daemon=True).start()


def _call_skill(skill_name: str, params: Dict[str, Any],
                wait_sec: float) -> dict:
    """Send one ExecuteSkill goal and return its flattened result dict."""
    with _lock:
        _ensure_ros()
        if not _client.wait_for_server(timeout_sec=5.0):
            return {"success": False,
                    "message": "/execute_skill action server not available "
                               "(is the robot pipeline running?)"}
        goal = ExecuteSkill.Goal()
        goal.skill = skill_name
        goal.params_json = json.dumps(params)

        send = _client.send_goal_async(goal)
        while not send.done():
            time.sleep(0.05)
        gh = send.result()
        if gh is None or not gh.accepted:
            return {"success": False, "message": "goal rejected"}
        res_future = gh.get_result_async()
        deadline = time.time() + wait_sec + 30.0
        while not res_future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not res_future.done():
            return {"success": False,
                    "message": "timed out waiting for action result"}
        res = res_future.result().result
        out = {"success": bool(res.success), "message": str(res.message)}
        try:
            data = json.loads(res.result_json) if res.result_json else {}
            if isinstance(data, dict):
                out.update(data)
        except json.JSONDecodeError:
            pass
        return out


def _make_tool(skill):
    """Build a typed MCP tool function for one skill from its param schema."""
    def tool(**kwargs):
        # drop unset optionals left as None so the server applies its defaults
        params = {k: v for k, v in kwargs.items() if v is not None}
        wait = float(params.get("timeout_sec") or 0.0) or 180.0
        return _call_skill(skill.name, params, wait)

    sig_params = []
    annotations: Dict[str, Any] = {}
    for p in skill.params:
        default = inspect.Parameter.empty if p.required else p.default
        sig_params.append(inspect.Parameter(
            p.name, inspect.Parameter.KEYWORD_ONLY,
            default=default, annotation=p.py_type()))
        annotations[p.name] = p.py_type()
    annotations["return"] = dict
    tool.__signature__ = inspect.Signature(sig_params, return_annotation=dict)
    tool.__annotations__ = annotations
    tool.__name__ = skill.name
    tool.__doc__ = _build_doc(skill)
    return tool


def _build_doc(skill) -> str:
    lines = [skill.description, ""]
    if skill.params:
        lines.append("Args:")
        for p in skill.params:
            req = "required" if p.required else f"default={p.default!r}"
            lines.append(f"    {p.name} ({p.type}, {req}): {p.description}")
    lines.append("")
    lines.append("Returns a dict: success, message, plus skill-specific outputs.")
    return "\n".join(lines)


# Generate one tool per registered skill at import time.
for _skill in all_skills():
    mcp.tool(name=_skill.name)(_make_tool(_skill))


if __name__ == "__main__":
    mcp.run()
