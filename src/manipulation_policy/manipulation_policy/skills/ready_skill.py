# Copyright 2026 MobileManipulationCore Contributors
# Licensed under the Apache License, Version 2.0.
"""Ready skill — move the arm to the look-down capture ('ready') pose.

Companion to the 'arm_home_pose' skill: where 'arm_home_pose' parks the arm at
the zero pose, 'arm_ready_pose' raises it into the look-down capture pose used
before a pick. The pose is read from the 'capture_pose' server param so it stays
in sync with the pick and handover skills (no separate copy to drift out of date).
"""
from __future__ import annotations

from typing import Any, Callable, Dict

from .base import Skill, SkillParam, SkillResult, SkillContext
from .registry import register_skill


@register_skill
class ReadySkill(Skill):
    name = "arm_ready_pose"
    description = (
        "Move the robot arm to the 'ready' pose: the look-down capture pose "
        "used before a pick. Use this to raise the arm from rest/zero into a "
        "working configuration."
    )
    params = [
        SkillParam("time_sec", "number", default=5.0,
                   description="seconds to take for the move."),
    ]

    def execute(self, ctx: SkillContext, params: Dict[str, Any],
                feedback: Callable[[str, float], None],
                is_cancelled: Callable[[], bool]) -> SkillResult:
        time_sec = float(params["time_sec"])

        ready_pose = list(ctx.get_param("capture_pose",
                                        [0.0, 1.2, -0.2, 0.0, -0.35, 0.0]))

        feedback("MOVING", 0.1)
        ctx.log(f"[READY] moving to ready pose over {time_sec:.0f}s")
        ok = ctx.move_arm_to(ready_pose, time_sec=time_sec)
        feedback("DONE" if ok else "FAILED", 1.0)
        return SkillResult(
            ok,
            "reached 'ready'" if ok else "arm move to 'ready' failed",
            {"pose": "ready"})
