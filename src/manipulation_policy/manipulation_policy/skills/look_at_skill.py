# Copyright 2026 MobileManipulationCore Contributors
# Licensed under the Apache License, Version 2.0.
"""look_at skill — aim the wrist camera (D405) at a 3D point.

Where ``arm_ready_pose`` raises the arm to a *fixed* look-down capture pose, this
skill aims the wrist camera at an *arbitrary* point P so that point lands in the
wrist camera's view — the precondition for ``localize_object`` (camera="wrist")
on a target that isn't already framed by the static ready pose.

Two modes (selected by ``standoff_m``):
  * orientation-only (``standoff_m <= 0``): keep the camera where it is and only
    re-aim it at P (a light wrist twist from a working pose).
  * standoff (``standoff_m > 0``): first move the camera to ``standoff_m`` metres
    from P along the line from P toward the current camera, then aim at P.

How it works
------------
This is a genuine 6-DOF Cartesian-orientation task, and the visual-servo node's
external-target mode is position-only (it always commands identity orientation),
so look-at is solved with real IK:

  1. Resolve P into the arm base frame (``piper_base_link``).
  2. Pick the camera position C (current, or a standoff from P).
  3. Build the camera-optical orientation whose +Z (REP-104 optical forward)
     points from C toward P (a gluLookAt construction; world-up resolves the free
     roll about the look axis).
  4. Convert that desired *optical-frame* pose into the desired *TCP* pose using
     the static TF camera_optical <- tcp (MoveIt plans for the TCP, not the
     camera, which is a separate fixed branch off link6).
  5. Solve IK with MoveIt's ``/compute_ik`` (via ``ctx.compute_ik``) and execute
     the joint solution with ``ctx.move_arm_to``.

Reliability + safety
--------------------
* The camera roll about the look axis is free, so we try several roll angles
  (upright first) and take the first solution that keeps the move modest — this
  greatly improves KDL's hit rate and keeps the image near-upright.
* ``/compute_ik`` only checks the *goal* state (joint limits + collision), not
  the swept path, and ``move_arm_to`` then executes it open-loop. So we reject
  solutions whose joint move is large (``look_at_max_joint_step_rad``) — a big
  blind sweep should go through a planned/raised pose (ReadyArm) instead.
* A light plausibility gate rejects non-finite / out-of-reach targets; the
  IK reach check vets the rest. Needs move_group (MoveIt) running.
"""
from __future__ import annotations

import math
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np

from .base import Skill, SkillParam, SkillResult, SkillContext
from .registry import register_skill
from .at_pose_skills import _position, _to_arm_base, _implausible, _fmt


# --------------------------------------------------------------------------- #
# small pure-numpy geometry helpers (no ROS, so they unit-test trivially)
# --------------------------------------------------------------------------- #
def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else v


def _look_at_rotation(z_dir: np.ndarray, up: np.ndarray) -> np.ndarray:
    """Rotation (optical->base) whose +Z points along ``z_dir``.

    REP-104 optical convention: +X right, +Y down, +Z forward. With a world-up
    hint this yields an upright image. Columns are the optical axes in base.

        z = normalize(z_dir)            # forward (toward the target)
        x = normalize(z x up)           # right
        y = z x x                       # down

    A near-vertical look (z parallel to up) is degenerate; fall back to base +X,
    then base +Y, as the up hint.
    """
    z = _unit(np.asarray(z_dir, dtype=float))
    up = np.asarray(up, dtype=float)
    x = np.cross(z, up)
    if float(np.linalg.norm(x)) < 1e-6:
        x = np.cross(z, np.array([1.0, 0.0, 0.0]))
        if float(np.linalg.norm(x)) < 1e-6:
            x = np.cross(z, np.array([0.0, 1.0, 0.0]))
    x = _unit(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def _rot_z(theta: float) -> np.ndarray:
    """Rotation about +Z (the optical look axis), to spin roll without changing aim."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _quat_to_rotation(q: Tuple[float, float, float, float]) -> np.ndarray:
    """Quaternion (x, y, z, w) -> 3x3 rotation matrix."""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def _rotation_to_quat(r: np.ndarray) -> Tuple[float, float, float, float]:
    """3x3 rotation matrix -> quaternion (x, y, z, w). Standard branch-by-trace."""
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def _make_tf(trans: np.ndarray, rot: np.ndarray) -> np.ndarray:
    """Homogeneous 4x4 from a translation (3,) and rotation (3x3)."""
    t = np.eye(4)
    t[:3, :3] = rot
    t[:3, 3] = trans
    return t


# --------------------------------------------------------------------------- #
# skill
# --------------------------------------------------------------------------- #
@register_skill
class LookAtSkill(Skill):
    name = "look_at"
    description = (
        "Aim the wrist camera (D405) at a GIVEN 3D point so that point lands in "
        "the wrist camera's view — the precondition for localize_object with "
        "camera='wrist'. The point is [x, y, z] in base_footprint by default; "
        "pass 'frame' for another TF frame. By default it only re-orients from "
        "the current camera position; pass standoff_m > 0 to first move the "
        "camera that far from the point. Uses MoveIt IK, so move_group must be "
        "running. After look_at, run localize_object(camera='wrist')."
    )
    params = [
        SkillParam("position", "array", required=True,
                   description="point [x, y, z] in metres to look at (base_footprint "
                               "by default; see 'frame')."),
        SkillParam("frame", "string", default="base_footprint",
                   description="TF frame of 'position' (default base_footprint; the "
                               "arm base frame skips the transform)."),
        SkillParam("standoff_m", "number", default=0.0,
                   description="if >0, move the camera to this distance from the "
                               "point before aiming; <=0 aims from the current "
                               "camera position (orientation only)."),
        SkillParam("time_sec", "number", default=4.0,
                   description="seconds to take for the arm move."),
    ]

    # Per-skill tunables (auto-declared on the skill_server, runtime-settable).
    server_params = {
        "look_at_planning_group": "piper_arm",
        "look_at_eef_link": "piper_tcp",
        "look_at_optical_frame": "piper_camera_optical_frame",
        # plausibility bound on the target distance from the arm base (m).
        "look_at_max_reach_m": 1.2,
        # don't put the lens on top of the object in standoff mode (m).
        "look_at_min_standoff_m": 0.10,
        # per-IK-attempt solve time (s).
        "look_at_ik_timeout_sec": 1.0,
        # roll angles (deg) about the look axis to try, upright first — the camera
        # roll is free, so sampling it lifts IK success and keeps the image level.
        "look_at_roll_candidates_deg": [0.0, 30.0, -30.0, 60.0, -60.0,
                                        90.0, -90.0, 180.0],
        # up-vector hint (arm base frame) that resolves the camera roll.
        "look_at_up_axis": [0.0, 0.0, 1.0],
        # reject an IK solution that moves any joint more than this (rad): a big
        # blind move should go via a planned/raised pose, not open-loop.
        "look_at_max_joint_step_rad": 1.5,
        # accept the first (upright-first) solution whose largest joint move is
        # this small; otherwise scan all rolls and take the smallest-motion one.
        "look_at_comfortable_joint_step_rad": 0.5,
        # post-move tolerance (rad): the arm must end within this of the solution
        # or the look-at is reported failed (open-loop execution isn't trusted).
        "look_at_reach_tol_rad": 0.15,
    }

    def execute(self, ctx: SkillContext, params: Dict[str, Any],
                feedback: Callable[[str, float], None],
                is_cancelled: Callable[[], bool]) -> SkillResult:
        base = str(ctx.get_param("arm_base_frame", "piper_base_link"))
        group = str(ctx.get_param("look_at_planning_group", "piper_arm"))
        eef = str(ctx.get_param("look_at_eef_link", "piper_tcp"))
        optical = str(ctx.get_param("look_at_optical_frame",
                                    "piper_camera_optical_frame"))
        max_reach = float(ctx.get_param("look_at_max_reach_m", 1.2))
        min_standoff = float(ctx.get_param("look_at_min_standoff_m", 0.10))
        ik_timeout = float(ctx.get_param("look_at_ik_timeout_sec", 1.0))
        max_step = float(ctx.get_param("look_at_max_joint_step_rad", 1.5))
        comfortable_step = float(ctx.get_param(
            "look_at_comfortable_joint_step_rad", 0.5))
        reach_tol = float(ctx.get_param("look_at_reach_tol_rad", 0.15))
        up_axis = list(ctx.get_param("look_at_up_axis", [0.0, 0.0, 1.0]))
        rolls_deg = list(ctx.get_param("look_at_roll_candidates_deg",
                                       [0.0, 30.0, -30.0, 60.0, -60.0,
                                        90.0, -90.0, 180.0]))
        time_sec = float(params["time_sec"])
        standoff_req = float(params["standoff_m"])
        arm_joints = list(ctx.get_param(
            "arm_joints", [f"piper_joint{i}" for i in range(1, 7)]))

        # --- target into the arm base frame + plausibility gate ---
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
            return SkillResult(False, f"look-at point {_fmt(target)} is implausible "
                               f"({bad})", {"target": target})
        p_b = np.asarray(target, dtype=float)

        # --- current wrist-camera pose + static camera_optical <- tcp ---
        tf_base_opt = ctx.lookup_transform(base, optical)
        tf_opt_tcp = ctx.lookup_transform(optical, eef)
        if tf_base_opt is None or tf_opt_tcp is None:
            return SkillResult(False, "wrist-camera TF unavailable "
                               f"({base}<->{optical}<->{eef}); is "
                               "robot_state_publisher up?", {})
        c_cur = np.asarray(tf_base_opt[0], dtype=float)
        t_opt_tcp = _make_tf(np.asarray(tf_opt_tcp[0], dtype=float),
                             _quat_to_rotation(tf_opt_tcp[1]))

        # --- choose the camera position C and the look direction z ---
        if standoff_req > 0.0:
            d = max(standoff_req, min_standoff)
            back = c_cur - p_b                       # back off toward current cam
            if float(np.linalg.norm(back)) < 1e-3:   # camera ~ on the point
                back = c_cur if float(np.linalg.norm(c_cur)) > 1e-3 \
                    else np.array([1.0, 0.0, 0.0])
            c_b = p_b + d * _unit(back)
            mode = "standoff"
        else:
            c_b = c_cur
            d = float(np.linalg.norm(c_cur - p_b))
            mode = "orientation_only"

        look = p_b - c_b
        if float(np.linalg.norm(look)) < 1e-4:
            return SkillResult(False, "the target coincides with the camera "
                               "position; nothing to look at", {"target": target})
        z_dir = _unit(look)
        r_base_opt0 = _look_at_rotation(z_dir, np.asarray(up_axis, dtype=float))

        # current arm config (for the joint-move guard, minimal-motion pick and
        # the post-move reach check). Fail closed: a real arm always publishes
        # /joint_states, so a missing one means we cannot vet or verify the move.
        cur = ctx.current_joint_positions()
        if not cur or not all(j in cur for j in arm_joints):
            return SkillResult(False, "current joint state unavailable "
                               "(/joint_states not seen) — cannot safely solve or "
                               "verify a look-at move", {"target": target})
        cur_arm = [float(cur[j]) for j in arm_joints]

        # Fail fast (once) if MoveIt's IK service isn't up, rather than re-paying
        # the connect wait for every roll candidate below.
        if not ctx.ik_available():
            return SkillResult(False, "MoveIt /compute_ik is unavailable — is "
                               "move_group running? (look_at needs MoveIt for IK)",
                               {"target": target, "mode": mode})

        feedback("AIMING", 0.1)
        ctx.log(f"[LOOK_AT] target={_fmt(target)} mode={mode} standoff={d:.3f}m "
                f"cam={_fmt(list(c_b))}")
        if is_cancelled():
            return SkillResult(False, "canceled before solving IK", {"target": target})

        # --- try roll candidates (upright first). Accept the first whose joint
        #     move is comfortably small; else keep all reachable ones and take the
        #     smallest move. Reject everything if even the smallest exceeds the
        #     open-loop guard. ---
        chosen: Optional[Dict[str, Any]] = None
        within: List[Dict[str, Any]] = []           # reachable, within the guard
        best_over: Optional[Dict[str, Any]] = None  # reachable but too-big move
        any_solution = False
        for roll_deg in rolls_deg:
            if is_cancelled():
                return SkillResult(False, "canceled while solving IK",
                                   {"target": target})
            r_base_opt = r_base_opt0 @ _rot_z(math.radians(roll_deg))
            t_base_tcp = _make_tf(c_b, r_base_opt) @ t_opt_tcp
            tcp_pos = t_base_tcp[:3, 3]
            tcp_quat = _rotation_to_quat(t_base_tcp[:3, :3])
            sol = ctx.compute_ik(group, eef, tuple(float(v) for v in tcp_pos),
                                 tcp_quat, base, timeout=ik_timeout)
            if not sol or not all(j in sol for j in arm_joints):
                continue
            any_solution = True
            sol_arm = [float(sol[j]) for j in arm_joints]
            max_d = max(abs(a - b) for a, b in zip(sol_arm, cur_arm))
            cand = {"roll_deg": float(roll_deg), "joints": sol_arm,
                    "max_joint_step": max_d,
                    "tcp_target": {"position": [round(float(v), 4) for v in tcp_pos],
                                   "orientation": [round(float(v), 4) for v in tcp_quat]}}
            if max_d > max_step:
                if best_over is None or max_d < best_over["max_joint_step"]:
                    best_over = cand
                continue
            if max_d <= comfortable_step:        # upright-first fast path
                chosen = cand
                break
            within.append(cand)
        if chosen is None and within:            # none comfortable -> smallest move
            chosen = min(within, key=lambda c: c["max_joint_step"])

        if chosen is None:
            if best_over is not None:
                return SkillResult(
                    False,
                    f"look-at requires a large arm move (max joint move "
                    f"{best_over['max_joint_step']:.2f} rad > "
                    f"{max_step:.2f}); raise the arm (ReadyArm) or reduce the "
                    "standoff first",
                    {"target": target, "mode": mode,
                     "tcp_target": best_over["tcp_target"]})
            return SkillResult(False, f"cannot look at {_fmt(target)}: no reachable "
                               "IK solution for this view (try a different standoff)",
                               {"target": target, "mode": mode})

        result = {
            "target": [round(float(v), 4) for v in target],
            "frame": base,
            "mode": mode,
            "standoff_m": round(d, 4),
            "camera_position": [round(float(v), 4) for v in c_b],
            "roll_deg": round(chosen["roll_deg"], 1),
            "tcp_target": chosen["tcp_target"],
            "joint_solution": [round(v, 4) for v in chosen["joints"]],
        }

        # --- execute, then verify the arm actually reached the pose (move_arm_to
        #     reports completion, not success, so confirm via joint readback). ---
        feedback("MOVING", 0.6)
        ctx.log(f"[LOOK_AT] roll={chosen['roll_deg']:.0f}deg "
                f"joints={[round(v, 3) for v in chosen['joints']]}")
        if not ctx.move_arm_to(chosen["joints"], time_sec=time_sec):
            feedback("FAILED", 1.0)
            return SkillResult(False, f"solved a look-at pose for {_fmt(target)} "
                               "but the arm move was rejected/timed out", result)

        ctx.sleep(0.2)                           # let the final /joint_states arrive
        cur2 = ctx.current_joint_positions()
        if cur2 and all(j in cur2 for j in arm_joints):
            reach_err = max(abs(float(cur2[j]) - s)
                            for j, s in zip(arm_joints, chosen["joints"]))
            result["reach_error_rad"] = round(reach_err, 4)
            if reach_err > reach_tol:
                feedback("FAILED", 1.0)
                return SkillResult(False, f"the arm did not reach the look-at pose "
                                   f"for {_fmt(target)} (off by {reach_err:.2f} rad "
                                   f"> {reach_tol:.2f}); the camera may not be aimed",
                                   result)

        feedback("DONE", 1.0)
        return SkillResult(True, f"aimed the wrist camera at {_fmt(target)}; the "
                           "point should now be in the wrist view — run "
                           "localize_object(camera='wrist')", result)
