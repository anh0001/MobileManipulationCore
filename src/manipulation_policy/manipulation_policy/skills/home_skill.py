# Copyright 2026 MobileManipulationCore Contributors
# Licensed under the Apache License, Version 2.0.
"""Home skill — move the arm to a named joint pose.

Example of a second, non-grasp skill. Demonstrates how little a new skill needs:
declare params, implement execute, register. No new message, no new node, no
launch edits, no MCP edits — it appears as an MCP tool automatically.

Named poses are read from the server param 'named_poses' (a flat dict-like list
of name -> 6 joint values is awkward in ROS params, so poses are passed as
individual params named pose.<name>). The built-in fallback is 'ready' =
capture_pose and 'rest' = all zeros.
"""
from __future__ import annotations

from typing import Any, Callable, Dict

from .base import Skill, SkillParam, SkillResult, SkillContext
from .registry import register_skill


@register_skill
class HomeSkill(Skill):
    name = "home"
    description = (
        "Move the robot arm to a named joint pose. 'ready' is the look-down "
        "capture pose used before a pick; 'rest' folds the arm to zeros. Use "
        "this to reset the arm between tasks or park it safely."
    )
    params = [
        SkillParam("pose", "string", default="ready",
                   description="named pose: 'ready' (look-down capture) or "
                               "'rest' (all joints zero)."),
        SkillParam("time_sec", "number", default=5.0,
                   description="seconds to take for the move."),
    ]

    def execute(self, ctx: SkillContext, params: Dict[str, Any],
                feedback: Callable[[str, float], None],
                is_cancelled: Callable[[], bool]) -> SkillResult:
        pose_name = str(params["pose"]).strip().lower()
        time_sec = float(params["time_sec"])

        capture_pose = list(ctx.get_param("capture_pose",
                                          [0.0, 1.2, -0.2, 0.0, -0.35, 0.0]))
        poses = {
            "ready": capture_pose,
            "rest": [0.0] * len(capture_pose),
        }
        if pose_name not in poses:
            return SkillResult(False,
                               f"unknown pose '{pose_name}'; "
                               f"choices: {', '.join(sorted(poses))}", {})

        feedback("MOVING", 0.1)
        ctx.log(f"[HOME] moving to '{pose_name}' over {time_sec:.0f}s")
        ok = ctx.move_arm_to(poses[pose_name], time_sec=time_sec)
        feedback("DONE" if ok else "FAILED", 1.0)
        return SkillResult(
            ok,
            f"reached '{pose_name}'" if ok else f"arm move to '{pose_name}' failed",
            {"pose": pose_name})
