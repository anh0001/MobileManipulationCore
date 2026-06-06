# Copyright 2026 MobileManipulationCore Contributors
# Licensed under the Apache License, Version 2.0.
"""Position-based pick / place skills — act at a GIVEN 3D point, no detection.

Companions to the detection-based pick / place / pick_and_place skills, for when
the caller already knows WHERE to act (e.g. a point returned by the
``localize_object`` skill). Instead of an open-vocabulary label they take a 3D
position and drive the arm straight to it:

  * ``pick_at``           — open the gripper, descend onto the point, close (grasp).
  * ``place_at``          — release the held object at the point (from a standoff).
  * ``pick_and_place_at`` — grasp at one point, then release at another.

How it works
------------
The visual-servo node already has an *external target* mode (``use_external_target``
+ ``external_grasp_target`` / ``external_pregrasp_target``, base frame): when set it
skips detection and drives GUARDED_APPROACH straight to the injected pose. Whether
it grasps (descend + CLOSE) or places (standoff + LIFT + OPEN) is decided by
``place_mode``. These skills are thin wrappers around that path — the same one
``pick_and_place`` already uses for its blind place.

Frames
------
The external target is in the arm base frame (``piper_base_link``). Callers give
the point in ``base_footprint`` by default (what ``localize_object`` returns) and
the skill transforms it via TF; pass ``frame`` to use another TF frame, or the arm
base frame directly to skip the transform.

Safety
------
There is NO detection or plane fit here — the caller's coordinate is trusted.
A light plausibility gate rejects non-finite / out-of-reach points, and the
node's own reach/workspace guard aborts (LOST) anything it cannot reach. Give a
point from ``localize_object`` (or a measured one); a bad coordinate drives the
arm to a bad place. ``pick_at`` opens the gripper itself first, because the
external path bypasses the detection pipeline's OPEN_GRIPPER step.
"""
from __future__ import annotations

import math
from typing import Any, Callable, Dict, List, Optional, Tuple

from .base import Skill, SkillParam, SkillResult, SkillContext
from .registry import register_skill
from ._motion import lift_to_ready, PICK_LIFT_PARAMS

# Servo states that confirm a fresh external GRASP is actually running...
PICK_ACTIVE_STATES = {"GUARDED_APPROACH", "APPROACH_DEPTH", "CLOSE_GRIPPER", "LIFT"}
# ...and a blind PLACE (external mode skips ACQUIRE/CLOSE).
PLACE_ACTIVE_STATES = {"GUARDED_APPROACH", "LIFT", "OPEN_GRIPPER"}

AT_POSE_PARAMS: Dict[str, Any] = {
    **PICK_LIFT_PARAMS,                 # post-grasp wrist-up lift then ready pose
    "pick_at_standoff_m": 0.12,         # pregrasp height above the grasp point
    "place_at_clearance_m": 0.15,       # standoff height above the place point
    "at_pose_max_reach_m": 1.2,         # plausibility gate: max dist from arm base
}


# --------------------------------------------------------------------------- #
# shared helpers
# --------------------------------------------------------------------------- #
def _position(params: Dict[str, Any], key: str) -> Optional[List[float]]:
    """Read a length-3 [x, y, z] position param; None if malformed."""
    v = params.get(key)
    if not isinstance(v, (list, tuple)) or len(v) != 3:
        return None
    try:
        return [float(v[0]), float(v[1]), float(v[2])]
    except (TypeError, ValueError):
        return None


def _to_arm_base(ctx: SkillContext, xyz: List[float], frame: str) -> Optional[List[float]]:
    """Transform a point into the arm base frame (the servo's external-target frame).

    ``frame`` empty or already the arm base frame -> returned unchanged. Otherwise
    a TF lookup (e.g. base_footprint -> piper_base_link); None if TF is unavailable.
    """
    base = str(ctx.get_param("arm_base_frame", "piper_base_link"))
    f = (frame or "").strip()
    if f in ("", base):
        return [float(v) for v in xyz]
    p = ctx.transform_point(base, f, (xyz[0], xyz[1], xyz[2]))
    return [float(p[0]), float(p[1]), float(p[2])] if p is not None else None


def _implausible(xyz: List[float], max_reach: float) -> Optional[str]:
    """Light sanity gate on an arm-base point. Returns a reason or None.

    Catches gross garbage (NaN/inf, or a point absurdly far from the arm base)
    before committing the arm to it. The node's reach guard vets the rest.
    """
    if len(xyz) != 3 or not all(math.isfinite(v) for v in xyz):
        return "non-finite coordinate"
    x, y, z = xyz
    if math.sqrt(x * x + y * y + z * z) > max_reach:
        return f"out of reach (>{max_reach:.2f} m from the arm base)"
    return None


def _fmt(xyz: List[float]) -> str:
    return "(" + ", ".join(f"{v:.3f}" for v in xyz) + ")"


def _gate_off(ctx: SkillContext) -> None:
    """Hard-stop the servo and clear the external-target / place flags."""
    ctx.set_bool_param("grasp_enabled", False)
    ctx.set_bool_param("grasp_auto_loop", False)
    ctx.set_bool_param("place_mode", False)
    ctx.set_bool_param("use_external_target", False)


def _drive_external_target(
        ctx: SkillContext, feedback: Callable[[str, float], None],
        is_cancelled: Callable[[], bool], grasp_xyz: List[float],
        pregrasp_xyz: List[float], place_mode: bool, timeout: float,
        active_states: set, label: str, prog0: float, prog1: float) -> Tuple[bool, str]:
    """Inject a base-frame external target and run the servo to it.

    place_mode=False -> descend to ``grasp_xyz`` and CLOSE (grasp).
    place_mode=True  -> reach the ``pregrasp_xyz`` standoff, LIFT, OPEN (release).
    Returns (ok, message). Mirrors pick_and_place._blind_place's external path.
    """
    ctx.set_bool_param("grasp_enabled", False)
    ctx.set_bool_param("place_mode", bool(place_mode))
    ctx.set_double_array_param("external_grasp_target", grasp_xyz)
    ctx.set_double_array_param("external_pregrasp_target", pregrasp_xyz)
    ctx.set_bool_param("use_external_target", True)
    ctx.publish_prompt("")                       # detector dormant in external mode
    if not ctx.set_bool_param("grasp_enabled", True):
        return (False, "could not reach the visual-servo node")
    ctx.set_bool_param("grasp_auto_loop", True)  # bump a stale DONE -> IDLE once

    t0 = ctx.now()
    saw = False
    loop_disabled = False
    while ctx.ok():
        if is_cancelled():
            return (False, "canceled")
        st = ctx.vs_state
        elapsed = ctx.now() - t0
        feedback(f"{label}:{st}",
                 min(prog1, prog0 + (prog1 - prog0) * elapsed / max(timeout, 1e-3)))
        if not loop_disabled and st not in ("", "DONE"):
            loop_disabled = ctx.set_bool_param("grasp_auto_loop", False)
        if st in active_states:
            saw = True
        if st == "LOST":
            return (False, "aborted (LOST) — the node's reach/workspace guard "
                           "rejected the target (out of reach or unsafe)")
        if saw and st == "DONE":
            ctx.set_bool_param("grasp_auto_loop", False)
            ctx.sleep(0.8)                       # let lift/width settle
            return (True, "done")
        if elapsed > timeout:
            return (False, f"timeout after {timeout:.0f}s in state '{st}'")
        ctx.sleep(0.1)
    return (False, "node shutting down")


# --------------------------------------------------------------------------- #
# pick_at
# --------------------------------------------------------------------------- #
@register_skill
class PickAtSkill(Skill):
    name = "pick_at"
    description = (
        "Grasp at a GIVEN 3D point (no detection). Opens the gripper, descends onto "
        "the point from a standoff above it, and closes to grasp. The point is "
        "[x, y, z] in base_footprint by default (e.g. from localize_object); pass "
        "'frame' to use another TF frame. Trusts the coordinate — there is no "
        "detection/plane fit, so give a point from localize_object or a measured "
        "one. Succeeds only if the post-lift gripper width shows something is held."
    )
    params = [
        SkillParam("position", "array", required=True,
                   description="grasp point [x, y, z] in metres (base_footprint by "
                               "default; see 'frame')."),
        SkillParam("frame", "string", default="base_footprint",
                   description="TF frame of 'position' (default base_footprint; "
                               "use the arm base frame to skip the transform)."),
        SkillParam("standoff_m", "number", default=0.0,
                   description="pregrasp height above the point; <=0 uses the server "
                               "pick_at_standoff_m."),
        SkillParam("timeout_sec", "number", default=0.0,
                   description="max seconds; <=0 uses the server default_timeout_sec."),
    ]
    server_params = AT_POSE_PARAMS

    def execute(self, ctx, params, feedback, is_cancelled):
        timeout = float(params["timeout_sec"]) or float(
            ctx.get_param("default_timeout_sec", 60.0))
        held_threshold = float(ctx.get_param("held_width_threshold", 0.004))
        capture_pose = list(ctx.get_param("capture_pose",
                                          [0.0, 1.2, -0.2, 0.0, -0.35, 0.0]))
        max_reach = float(ctx.get_param("at_pose_max_reach_m", 1.2))
        standoff = float(params["standoff_m"]) or float(
            ctx.get_param("pick_at_standoff_m", 0.12))

        pos = _position(params, "position")
        if pos is None:
            return SkillResult(False, "position must be [x, y, z]", {})
        grasp = _to_arm_base(ctx, pos, str(params["frame"]))
        if grasp is None:
            return SkillResult(False, f"could not transform the point from "
                               f"'{params['frame']}' into the arm base frame (TF "
                               "unavailable)", {})
        bad = _implausible(grasp, max_reach)
        if bad:
            return SkillResult(False, f"grasp point {_fmt(grasp)} is implausible "
                               f"({bad})", {"grasp_target": grasp})
        pregrasp = [grasp[0], grasp[1], grasp[2] + standoff]

        result = {"grasp_target": [round(v, 4) for v in grasp], "object_held": False,
                  "gripper_width": round(float(ctx.gripper_width), 4)}
        ctx.log(f"[PICK_AT] grasp={_fmt(grasp)} pregrasp={_fmt(pregrasp)} "
                f"(standoff={standoff:.3f} m)")

        # Clean, known state before driving joints (a leftover-enabled servo must
        # not fight the move), then start from the look-down ready pose and OPEN the
        # gripper — the external path goes straight to GUARDED_APPROACH and never
        # runs OPEN_GRIPPER itself.
        _gate_off(ctx)
        feedback("READY", 0.05)
        ctx.move_arm_to(capture_pose, time_sec=5.0)
        open_pos = float(ctx.get_param("gripper_open_position", 0.07))
        max_effort = float(ctx.get_param("gripper_max_effort", 5.0))
        ctx.set_gripper(open_pos, max_effort=max_effort)
        if is_cancelled():
            _gate_off(ctx)
            return SkillResult(False, "canceled before approach", result)

        ok, msg = _drive_external_target(
            ctx, feedback, is_cancelled, grasp, pregrasp, place_mode=False,
            timeout=timeout, active_states=PICK_ACTIVE_STATES, label="PICK_AT",
            prog0=0.15, prog1=0.85)
        if not ok:
            _gate_off(ctx)
            ctx.move_arm_to(capture_pose, time_sec=4.0)
            return SkillResult(False, f"grasp at {_fmt(grasp)} failed: {msg}", result)

        held = ctx.gripper_width > held_threshold
        result["object_held"] = bool(held)
        result["gripper_width"] = round(float(ctx.gripper_width), 4)

        feedback("RETURN_READY", 0.9)
        ctx.set_bool_param("grasp_enabled", False)
        lift_to_ready(ctx, feedback, label="RETURN_READY", progress=0.92)
        _gate_off(ctx)
        feedback("DONE", 1.0)
        if not held:
            return SkillResult(False, f"likely empty grasp at {_fmt(grasp)}; width="
                               f"{ctx.gripper_width:.4f} <= {held_threshold:.4f}", result)
        return SkillResult(True, f"grasped at {_fmt(grasp)}; width="
                           f"{ctx.gripper_width:.4f}; returned to ready", result)


# --------------------------------------------------------------------------- #
# place_at
# --------------------------------------------------------------------------- #
@register_skill
class PlaceAtSkill(Skill):
    name = "place_at"
    description = (
        "Release the held object at a GIVEN 3D point (no detection). Approaches a "
        "standoff above the point and opens the gripper to drop the object there. "
        "The point is [x, y, z] in base_footprint by default; pass 'frame' to use "
        "another TF frame. RUN ONLY AFTER A SUCCESSFUL pick/pick_at (must be holding "
        "something)."
    )
    params = [
        SkillParam("position", "array", required=True,
                   description="release point [x, y, z] in metres (base_footprint by "
                               "default; see 'frame')."),
        SkillParam("frame", "string", default="base_footprint",
                   description="TF frame of 'position' (default base_footprint)."),
        SkillParam("clearance_m", "number", default=0.0,
                   description="standoff height above the point at which to release; "
                               "<=0 uses the server place_at_clearance_m."),
        SkillParam("timeout_sec", "number", default=0.0,
                   description="max seconds; <=0 uses the server default_timeout_sec."),
    ]
    server_params = AT_POSE_PARAMS

    def execute(self, ctx, params, feedback, is_cancelled):
        timeout = float(params["timeout_sec"]) or float(
            ctx.get_param("default_timeout_sec", 60.0))
        held_threshold = float(ctx.get_param("held_width_threshold", 0.004))
        capture_pose = list(ctx.get_param("capture_pose",
                                          [0.0, 1.2, -0.2, 0.0, -0.35, 0.0]))
        max_reach = float(ctx.get_param("at_pose_max_reach_m", 1.2))
        clearance = float(params["clearance_m"]) or float(
            ctx.get_param("place_at_clearance_m", 0.15))

        if ctx.gripper_width <= held_threshold:
            return SkillResult(False, f"no object held (gripper width "
                               f"{ctx.gripper_width:.4f} <= {held_threshold:.4f}); "
                               "pick something first", {})
        pos = _position(params, "position")
        if pos is None:
            return SkillResult(False, "position must be [x, y, z]", {})
        target = _to_arm_base(ctx, pos, str(params["frame"]))
        if target is None:
            return SkillResult(False, f"could not transform the point from "
                               f"'{params['frame']}' into the arm base frame (TF "
                               "unavailable)", {})
        bad = _implausible(target, max_reach)
        if bad:
            return SkillResult(False, f"release point {_fmt(target)} is implausible "
                               f"({bad})", {"place_target": target})

        ok, msg = self._place(ctx, feedback, is_cancelled, target, clearance, timeout)
        _gate_off(ctx)
        ctx.move_arm_to(capture_pose, time_sec=5.0)
        feedback("DONE", 1.0)
        result = {"place_target": [round(v, 4) for v in target],
                  "released": bool(ok),
                  "gripper_width": round(float(ctx.gripper_width), 4)}
        if not ok:
            return SkillResult(False, f"place at {_fmt(target)} failed: {msg}", result)
        return SkillResult(True, f"released at {_fmt(target)}; returned to ready",
                           result)

    def _place(self, ctx, feedback, is_cancelled, target, clearance, timeout):
        """Release at ``target`` via the external-target place path."""
        pregrasp = [target[0], target[1], target[2] + clearance]
        ctx.log(f"[PLACE_AT] target={_fmt(target)} release-approach z={pregrasp[2]:.3f} "
                f"(clearance={clearance:.3f} m)")
        # Open right at the standoff (the clearance already lifts the object clear),
        # then restore the node's value for the standalone `place` skill.
        prev = ctx.get_remote_params(
            ["place_release_clearance_m"]).get("place_release_clearance_m")
        ctx.set_double_param("place_release_clearance_m", 0.0)
        try:
            return _drive_external_target(
                ctx, feedback, is_cancelled, target, pregrasp, place_mode=True,
                timeout=timeout, active_states=PLACE_ACTIVE_STATES, label="PLACE_AT",
                prog0=0.1, prog1=0.9)
        finally:
            if prev is not None:
                ctx.set_double_param("place_release_clearance_m", float(prev))


# --------------------------------------------------------------------------- #
# pick_and_place_at
# --------------------------------------------------------------------------- #
@register_skill
class PickAndPlaceAtSkill(Skill):
    name = "pick_and_place_at"
    description = (
        "Grasp at one GIVEN 3D point and release at another (no detection). Picks at "
        "'pick_position', lifts, then releases at 'place_position'. Both points are "
        "[x, y, z] in base_footprint by default; pass 'frame' to use another TF "
        "frame for both. Use when you already know both coordinates (e.g. from "
        "localize_object)."
    )
    params = [
        SkillParam("pick_position", "array", required=True,
                   description="grasp point [x, y, z] in metres (base_footprint by "
                               "default)."),
        SkillParam("place_position", "array", required=True,
                   description="release point [x, y, z] in metres (base_footprint by "
                               "default)."),
        SkillParam("frame", "string", default="base_footprint",
                   description="TF frame of both points (default base_footprint)."),
        SkillParam("timeout_sec", "number", default=0.0,
                   description="max seconds for EACH phase; <=0 uses the server "
                               "default_timeout_sec."),
    ]
    server_params = AT_POSE_PARAMS

    def execute(self, ctx, params, feedback, is_cancelled):
        timeout = float(params["timeout_sec"]) or float(
            ctx.get_param("default_timeout_sec", 60.0))
        held_threshold = float(ctx.get_param("held_width_threshold", 0.004))
        capture_pose = list(ctx.get_param("capture_pose",
                                          [0.0, 1.2, -0.2, 0.0, -0.35, 0.0]))
        max_reach = float(ctx.get_param("at_pose_max_reach_m", 1.2))
        frame = str(params["frame"])

        pick_pos = _position(params, "pick_position")
        place_pos = _position(params, "place_position")
        if pick_pos is None or place_pos is None:
            return SkillResult(False, "pick_position and place_position must each "
                               "be [x, y, z]", {})
        grasp = _to_arm_base(ctx, pick_pos, frame)
        target = _to_arm_base(ctx, place_pos, frame)
        if grasp is None or target is None:
            return SkillResult(False, f"could not transform the points from "
                               f"'{frame}' into the arm base frame (TF unavailable)",
                               {})
        for label, p in (("pick", grasp), ("place", target)):
            bad = _implausible(p, max_reach)
            if bad:
                return SkillResult(False, f"{label} point {_fmt(p)} is implausible "
                                   f"({bad})", {})

        result = {"grasp_target": [round(v, 4) for v in grasp],
                  "place_target": [round(v, 4) for v in target]}

        # ---- pick ----
        feedback("PICK", 0.0)
        _gate_off(ctx)                      # clean state before driving joints
        standoff = float(ctx.get_param("pick_at_standoff_m", 0.12))
        pregrasp = [grasp[0], grasp[1], grasp[2] + standoff]
        ctx.move_arm_to(capture_pose, time_sec=5.0)
        open_pos = float(ctx.get_param("gripper_open_position", 0.07))
        ctx.set_gripper(open_pos, max_effort=float(ctx.get_param("gripper_max_effort", 5.0)))
        ok, msg = _drive_external_target(
            ctx, feedback, is_cancelled, grasp, pregrasp, place_mode=False,
            timeout=timeout, active_states=PICK_ACTIVE_STATES, label="PICK",
            prog0=0.05, prog1=0.5)
        held = ok and ctx.gripper_width > held_threshold
        result["object_held"] = bool(held)
        result["gripper_width"] = round(float(ctx.gripper_width), 4)
        if not held:
            _gate_off(ctx)
            ctx.move_arm_to(capture_pose, time_sec=4.0)
            why = msg if not ok else (f"empty grasp; width={ctx.gripper_width:.4f}")
            return SkillResult(False, f"pick at {_fmt(grasp)} failed, not placing: "
                               f"{why}", result)

        # ---- lift to transit, then place ----
        feedback("LIFT_TRANSIT", 0.5)
        ctx.set_bool_param("grasp_enabled", False)
        lift_to_ready(ctx, feedback, label="LIFT_TRANSIT", progress=0.5)

        feedback("PLACE", 0.6)
        clearance = float(ctx.get_param("place_at_clearance_m", 0.15))
        place_pregrasp = [target[0], target[1], target[2] + clearance]
        prev = ctx.get_remote_params(
            ["place_release_clearance_m"]).get("place_release_clearance_m")
        ctx.set_double_param("place_release_clearance_m", 0.0)
        try:
            placed, place_msg = _drive_external_target(
                ctx, feedback, is_cancelled, target, place_pregrasp, place_mode=True,
                timeout=timeout, active_states=PLACE_ACTIVE_STATES, label="PLACE",
                prog0=0.6, prog1=0.95)
        finally:
            if prev is not None:
                ctx.set_double_param("place_release_clearance_m", float(prev))
        _gate_off(ctx)
        ctx.move_arm_to(capture_pose, time_sec=5.0)
        result["released"] = bool(placed)
        result["gripper_width"] = round(float(ctx.gripper_width), 4)
        feedback("DONE", 1.0)
        if not placed:
            return SkillResult(False, f"grasped at {_fmt(grasp)} but place at "
                               f"{_fmt(target)} failed: {place_msg}", result)
        return SkillResult(True, f"grasped at {_fmt(grasp)} and released at "
                           f"{_fmt(target)}; returned to ready", result)
