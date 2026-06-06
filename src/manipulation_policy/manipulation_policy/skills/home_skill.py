# Copyright 2026 MobileManipulationCore Contributors
# Licensed under the Apache License, Version 2.0.
"""Home skill — park the arm at the zero (rest) joint pose.

Example of a second, non-grasp skill. Demonstrates how little a new skill needs:
declare params, implement execute, register. No new message, no new node, no
launch edits, no MCP edits — it appears as an MCP tool automatically.

'arm_home_pose' parks the arm at the zero pose (all joints zero) by default. The
companion 'arm_ready_pose' skill (skills/ready_skill.py) raises the arm into the
look-down capture pose. Both 'rest' (all zeros) and 'ready' (capture_pose, read
from the server param) remain selectable here via the 'pose' param.

LiDAR clearance: interpolating straight from a down-pointing pose to zero sweeps
the end of the arm through the Livox Mid-360. So when parking at zero we first
move through a 'lift' waypoint that pitches the wrist (joint5) up to raise the
EEF in +Z, then fold to zero. The waypoint and its duration are server params
('home_lift_pose', 'home_lift_time_sec') so they can be tuned on the real robot;
set 'home_lift_pose' to all zeros to disable the lift and move straight to zero.
"""
from __future__ import annotations

from typing import Any, Callable, Dict

from .base import Skill, SkillParam, SkillResult, SkillContext
from .registry import register_skill


@register_skill
class HomeSkill(Skill):
    name = "arm_home_pose"
    description = (
        "Move the robot arm to a named joint pose. Defaults to 'rest', the "
        "zero pose with every joint at zero — use it to park the arm safely "
        "between tasks. Parking at zero first lifts the wrist up so the EEF "
        "clears the Mid-360 LiDAR, then folds to zero. 'ready' is the look-down "
        "capture pose (the dedicated 'arm_ready_pose' skill does the same move)."
    )
    params = [
        SkillParam("pose", "string", default="rest",
                   description="named pose: 'rest' (all joints zero, default) "
                               "or 'ready' (look-down capture)."),
        SkillParam("time_sec", "number", default=5.0,
                   description="seconds to take for the move (the fold to zero)."),
    ]

    # Per-skill tunables (auto-declared on the skill_server, runtime-settable).
    # 'home_lift_pose' is the LiDAR-clearance waypoint the arm passes through on
    # its way to zero: wrist (joint5) pitched up to raise the EEF in +Z, every
    # other joint at zero. On this arm POSITIVE joint5 pitches the wrist DOWN, so
    # lifting up needs a NEGATIVE joint5 (-0.8 rad ~ -46deg, enough to clear the
    # LiDAR without over-rotating; well within the +-1.57 limit). Tune on the
    # real robot (the '7. Handover: Echo arm joints' task reads live joints); set
    # to all zeros to skip the lift.
    server_params = {
        "home_lift_pose": [0.0, 0.0, 0.0, 0.0, -0.8, 0.0],
        "home_lift_time_sec": 3.0,
    }

    def execute(self, ctx: SkillContext, params: Dict[str, Any],
                feedback: Callable[[str, float], None],
                is_cancelled: Callable[[], bool]) -> SkillResult:
        pose_name = str(params["pose"]).strip().lower()
        time_sec = float(params["time_sec"])

        capture_pose = list(ctx.get_param("capture_pose",
                                          [0.0, 1.2, -0.2, 0.0, -0.35, 0.0]))
        zero_pose = [0.0] * len(capture_pose)
        poses = {
            "ready": capture_pose,
            "rest": zero_pose,
        }
        if pose_name not in poses:
            return SkillResult(False,
                               f"unknown pose '{pose_name}'; "
                               f"choices: {', '.join(sorted(poses))}", {})
        target = poses[pose_name]

        # Returning to zero: lift the EEF clear of the LiDAR (wrist up, +Z) before
        # folding to zero. A lift pose that equals the target (e.g. all zeros) or
        # has the wrong length is treated as "disabled" -> a single direct move.
        if pose_name == "rest":
            lift_pose = list(ctx.get_param("home_lift_pose",
                                           [0.0, 0.0, 0.0, 0.0, -0.8, 0.0]))
            lift_time = float(ctx.get_param("home_lift_time_sec", 3.0))
            if len(lift_pose) == len(target) and lift_pose != target:
                feedback("LIFTING", 0.1)
                ctx.log(f"[HOME] lifting EEF clear of LiDAR (wrist up) "
                        f"over {lift_time:.0f}s")
                if not ctx.move_arm_to(lift_pose, time_sec=lift_time):
                    feedback("FAILED", 1.0)
                    return SkillResult(False,
                                       "lift-clear move failed before homing",
                                       {"stage": "lift"})
                if is_cancelled():
                    return SkillResult(False,
                                       "cancelled after lift, before homing",
                                       {"stage": "lift"})

        feedback("MOVING", 0.6 if pose_name == "rest" else 0.1)
        ctx.log(f"[HOME] moving to '{pose_name}' over {time_sec:.0f}s")
        ok = ctx.move_arm_to(target, time_sec=time_sec)
        feedback("DONE" if ok else "FAILED", 1.0)
        return SkillResult(
            ok,
            f"reached '{pose_name}'" if ok else f"arm move to '{pose_name}' failed",
            {"pose": pose_name})
