# Copyright 2026 MobileManipulationCore Contributors
# Licensed under the Apache License, Version 2.0.
"""Shared arm-motion helpers used by more than one skill.

Keeping the post-grasp return in one place stops the pick and pick_and_place
skills from drifting apart.
"""
from __future__ import annotations

import math
from typing import Callable

from .base import SkillContext

# Defaults mirror the skill_server param defaults so the helper still does the
# right thing if a param is somehow unset. The ready/capture pose is the
# look-down pose; the lift pose keeps the arm forward (j2) but pitches the wrist
# up (NEGATIVE joint5 = up on this arm) so a held object tilts up and clears the
# robot body / Mid-360 LiDAR instead of being dragged down on the way back.
_DEFAULT_CAPTURE = [0.0, 1.2, -0.2, 0.0, -0.35, 0.0]
_DEFAULT_PICK_LIFT = [0.0, 1.2, -0.2, 0.0, -1.2, 0.0]

# Server params (declared by the pick / pick_and_place skills) this helper reads.
PICK_LIFT_PARAMS = {
    "pick_lift_pose": _DEFAULT_PICK_LIFT,
    "pick_lift_time_sec": 3.0,
    # Lift only when the EEF is within this horizontal distance (m) of the arm
    # base — i.e. close to the robot body/LiDAR. A far-forward reach can return
    # to ready without the lift, so it is skipped.
    "pick_lift_near_radius_m": 0.35,
}


def lift_to_ready(ctx: SkillContext,
                  feedback: Callable[[str, float], None],
                  *, label: str = "RETURN_READY", progress: float = 0.95) -> bool:
    """After a grasp, lift the wrist up (only if close to the robot), then ready.

    Two-stage so a held object near the robot never sweeps straight from the
    (low, down-pointing) grasp pose back to the ready pose:
      1. IF the live EEF is within ``pick_lift_near_radius_m`` of the arm base,
         move to ``pick_lift_pose`` — wrist pitched up, object raised/tilted clear;
      2. move to ``capture_pose`` — the look-down ready pose.

    The lift in step 1 is skipped when the EEF is far forward (a return that won't
    graze the body/LiDAR). If the EEF pose is unknown (no TF), it lifts anyway —
    the safe default.

    All knobs are server params (``pick_lift_pose``, ``pick_lift_time_sec``,
    ``pick_lift_near_radius_m``, ``capture_pose``), runtime-settable on
    /skill_server. Set ``pick_lift_pose`` equal to ``capture_pose`` (or give it the
    wrong length) to disable the lift entirely. Returns True only if every move
    reported done.
    """
    capture_pose = list(ctx.get_param("capture_pose", _DEFAULT_CAPTURE))
    lift_pose = list(ctx.get_param("pick_lift_pose", _DEFAULT_PICK_LIFT))
    lift_time = float(ctx.get_param("pick_lift_time_sec", 3.0))
    near_radius = float(ctx.get_param("pick_lift_near_radius_m", 0.35))

    feedback(label, progress)

    # Decide whether the EEF is close enough to the robot to need the lift.
    eef = ctx.eef_position()
    if eef is None:
        near = True
        ctx.log("[RETURN] EEF pose unknown -> lift (safe default)")
    else:
        horiz = math.hypot(eef[0], eef[1])
        near = horiz < near_radius
        ctx.log(f"[RETURN] EEF horiz dist {horiz:.3f} m "
                f"{'<' if near else '>='} {near_radius:.3f} m -> "
                f"{'lift' if near else 'no lift (return straight to ready)'}")

    ok = True
    do_lift = (near and len(lift_pose) == len(capture_pose)
               and lift_pose != capture_pose)
    if do_lift:
        ctx.log("[RETURN] lifting wrist up before settling at ready pose")
        ok = ctx.move_arm_to(lift_pose, time_sec=lift_time)
    ok = ctx.move_arm_to(capture_pose, time_sec=5.0) and ok
    return ok
