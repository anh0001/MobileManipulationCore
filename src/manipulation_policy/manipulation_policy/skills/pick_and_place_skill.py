# Copyright 2026 MobileManipulationCore Contributors
# Licensed under the Apache License, Version 2.0.
"""Pick-and-place skill — localize destination first, then pick, then blind place.

Why this exists: the standalone `place` skill re-detects the destination at place
time, but by then the held object occludes the wrist camera, so detection is
unreliable and the place churns. This skill localizes BOTH the object and the
destination up front (gripper empty, look-down pose, camera clear), REMEMBERS the
destination's 3D location in the base frame, picks the object, then places it at the
remembered location with NO live re-detection at place time.

Sequence per goal:
  Phase 0 — capture destination (camera clear):
    publish the destination prompt, look down, gate on with place_mode (so the
    approach can never descend onto the destination). The visual-servo node mirrors
    its computed grasp/pregrasp target to readback params; as soon as it reports a
    valid target we snapshot it (base frame) and gate off — which now hard-aborts the
    servo. If nothing is detected within acquire_timeout, ABORT before picking.
  Phase 1 — pick object:
    run the normal visual-servo grasp on the object. If the post-lift width says the
    grasp is empty, ABORT without placing.
  Phase 2 — blind place:
    feed the remembered target back via the external-target params and run place_mode
    with detection skipped; the arm drives straight to the standoff above the
    destination and releases. Return to the ready pose.

Held detection is heuristic (no force sensor) — mirrors pick_skill.
"""
from __future__ import annotations

from typing import Any, Callable, Dict, List, Optional, Tuple

from .base import Skill, SkillParam, SkillResult, SkillContext
from .registry import register_skill
from ._motion import lift_to_ready, PICK_LIFT_PARAMS

# Grasp-sequence states that confirm a fresh PICK attempt is actually running.
PICK_ACTIVE_STATES = {
    "OPEN_GRIPPER", "GUARDED_APPROACH", "APPROACH_DEPTH", "CLOSE_GRIPPER", "LIFT",
}
# States that confirm the blind PLACE is running (external mode skips ACQUIRE).
PLACE_ACTIVE_STATES = {"GUARDED_APPROACH", "LIFT", "OPEN_GRIPPER"}


@register_skill
class PickAndPlaceSkill(Skill):
    name = "pick_and_place"
    description = (
        "Pick an object and place it into/onto a destination (e.g. a box). The "
        "destination is localized FIRST while the gripper is empty and the wrist "
        "camera is unobstructed, then remembered and reused for the place, so the "
        "held object never blocks detection at place time. Aborts before picking "
        "if either the object or the destination is not detected up front."
    )
    params = [
        SkillParam("object", "string", required=True,
                   description='label of the object to pick, e.g. "banana", '
                               '"toy banana", "bread".'),
        SkillParam("destination", "string", required=True,
                   description='label of where to place it, e.g. "white box", '
                               '"box", "plate". Localized first, camera clear.'),
        SkillParam("timeout_sec", "number", default=0.0,
                   description="max seconds for EACH phase; <=0 uses the server "
                               "default_timeout_sec."),
    ]
    # Post-grasp return tunables (wrist-up lift then ready pose); see
    # skills/_motion.lift_to_ready.
    server_params = dict(PICK_LIFT_PARAMS)

    def execute(self, ctx: SkillContext, params: Dict[str, Any],
                feedback: Callable[[str, float], None],
                is_cancelled: Callable[[], bool]) -> SkillResult:
        obj = str(params["object"]).strip()
        dest = str(params["destination"]).strip()
        req_timeout = float(params["timeout_sec"])
        timeout = req_timeout if req_timeout > 0 else float(
            ctx.get_param("default_timeout_sec", 60.0))
        acquire_timeout = float(ctx.get_param("acquire_timeout_sec", 12.0))
        held_threshold = float(ctx.get_param("held_width_threshold", 0.012))
        capture_pose = list(ctx.get_param("capture_pose",
                                          [0.0, 1.2, -0.2, 0.0, -0.35, 0.0]))

        def gate_off():
            # Clearing grasp_enabled now hard-stops the servo (mid-sequence abort),
            # so this both ungates and halts any in-progress motion.
            ctx.set_bool_param("grasp_enabled", False)
            ctx.set_bool_param("grasp_auto_loop", False)
            ctx.set_bool_param("place_mode", False)
            ctx.set_bool_param("use_external_target", False)

        def fail(message: str, data: Optional[Dict[str, Any]] = None) -> SkillResult:
            gate_off()
            ctx.publish_prompt("")
            out = {"gripper_width": round(float(ctx.gripper_width), 4)}
            if data:
                out.update(data)
            return SkillResult(False, message, out)

        if not obj:
            return SkillResult(False, "empty object label", {})
        if not dest:
            return SkillResult(False, "empty destination label", {})
        ctx.log(f"[PnP '{obj}' -> '{dest}'] starting (per-phase timeout={timeout:.0f}s)")

        # ---- Phase 0: capture the destination location (camera clear) ----------
        feedback("CAPTURE_DEST", 0.0)
        gate_off()
        # place_mode=true during capture is defense-in-depth: even if the abort is a
        # tick late, a place-mode approach only goes to the standoff ABOVE the box
        # and never descends to close on it.
        ctx.set_bool_param("place_mode", True)
        ctx.set_bool_param("use_external_target", False)
        ctx.set_bool_param("grasp_auto_loop", True)   # bump a stale DONE -> IDLE
        ctx.publish_prompt(dest)
        ctx.move_arm_to(capture_pose, time_sec=5.0)
        if not ctx.set_bool_param("grasp_enabled", True):
            return fail("could not reach visual-servo node to capture destination")

        captured = self._capture_target(
            ctx, feedback, is_cancelled, acquire_timeout, label=dest)
        if isinstance(captured, str):                 # error string
            ctx.move_arm_to(capture_pose, time_sec=4.0)
            return fail(captured)
        grasp_xyz, pregrasp_xyz = captured
        gate_off()                                    # hard-stops the capture motion
        ctx.publish_prompt("")
        ctx.log(f"[PnP] captured '{dest}': grasp={_fmt(grasp_xyz)} "
                f"pregrasp={_fmt(pregrasp_xyz)}")

        # Sanity check the captured pose before committing to a pick.
        bad = _implausible(grasp_xyz) or _implausible(pregrasp_xyz)
        if bad:
            ctx.move_arm_to(capture_pose, time_sec=4.0)
            return fail(f"captured destination pose for '{dest}' is implausible "
                        f"({bad}); aborting before pick")

        # ---- Phase 1: pick the object ------------------------------------------
        feedback("PICK", 0.2)
        held, pick_msg = self._pick(
            ctx, feedback, is_cancelled, obj, timeout, acquire_timeout, held_threshold)
        if not held:
            # Abort WITHOUT placing; pick already gated off on its failure paths.
            return fail(f"pick failed, not placing: {pick_msg}",
                        {"phase": "pick"})

        # ---- Lift to a safe transit height before crossing to the destination --
        # The post-pick LIFT only raises a few cm; lift the wrist up first so the
        # held object tilts clear, then cross over at the elevated look-down pose
        # so it clears the destination (and anything between) on the way. Gate off
        # first so the abort halts the servo (object stays held) and the joint
        # move does not fight a servo command.
        feedback("LIFT_TRANSIT", 0.55)
        ctx.set_bool_param("grasp_enabled", False)
        lift_to_ready(ctx, feedback, label="LIFT_TRANSIT", progress=0.55)

        # ---- Phase 2: blind place at the remembered destination ----------------
        feedback("PLACE", 0.6)
        placed, place_msg = self._blind_place(
            ctx, feedback, is_cancelled, grasp_xyz, timeout)
        gate_off()
        ctx.move_arm_to(capture_pose, time_sec=5.0)
        feedback("DONE", 1.0)
        if not placed:
            return SkillResult(False, f"picked '{obj}' but place failed: {place_msg}", {
                "phase": "place",
                "object_held": True,
                "grasp_target": [round(v, 4) for v in grasp_xyz],
                "gripper_width": round(float(ctx.gripper_width), 4),
            })
        return SkillResult(
            True,
            f"picked '{obj}' and released on '{dest}'; returned to ready",
            {
                "object_held": True,
                "grasp_target": [round(v, 4) for v in grasp_xyz],
                "gripper_width": round(float(ctx.gripper_width), 4),
            })

    # --- phase helpers ------------------------------------------------------

    def _capture_target(self, ctx, feedback, is_cancelled, acquire_timeout, label):
        """Wait for the visual servo to report a valid target, snapshot it.

        Returns (grasp_xyz, pregrasp_xyz) on success, or an error string.
        """
        t0 = ctx.now()
        loop_disabled = False
        while ctx.ok():
            if is_cancelled():
                return "canceled during destination capture"
            st = ctx.vs_state
            elapsed = ctx.now() - t0
            feedback(f"CAPTURE:{st}", min(0.18, 0.18 * elapsed / max(acquire_timeout, 1e-3)))
            if not loop_disabled and st not in ("", "DONE"):
                loop_disabled = ctx.set_bool_param("grasp_auto_loop", False)
            rp = ctx.get_remote_params(
                ["last_target_valid", "last_grasp_target", "last_pregrasp_target"])
            g = rp.get("last_grasp_target") or []
            p = rp.get("last_pregrasp_target") or []
            if rp.get("last_target_valid") and len(g) == 3 and len(p) == 3:
                return ([float(v) for v in g], [float(v) for v in p])
            if elapsed > acquire_timeout:
                return (f"destination '{label}' not detected within "
                        f"{acquire_timeout:.0f}s (not visible from the ready pose, "
                        "or out of the camera view)")
            ctx.sleep(0.1)
        return "node shutting down during destination capture"

    def _pick(self, ctx, feedback, is_cancelled, obj, timeout, acquire_timeout,
              held_threshold) -> Tuple[bool, str]:
        """Run the normal grasp pipeline on the object. Returns (held, message)."""
        capture_pose = list(ctx.get_param("capture_pose",
                                           [0.0, 1.2, -0.2, 0.0, -0.35, 0.0]))

        def pick_gate_off():
            ctx.set_bool_param("grasp_enabled", False)
            ctx.set_bool_param("grasp_auto_loop", False)

        ctx.set_bool_param("grasp_enabled", False)
        ctx.set_bool_param("place_mode", False)
        ctx.set_bool_param("use_external_target", False)
        ctx.set_bool_param("grasp_auto_loop", True)   # bump a stale DONE -> IDLE
        ctx.publish_prompt(obj)
        ctx.move_arm_to(capture_pose, time_sec=5.0)
        feedback(f"PICK:{ctx.vs_state}", 0.2)
        if not ctx.set_bool_param("grasp_enabled", True):
            pick_gate_off()
            return (False, "could not reach visual-servo node to enable grasp")

        t0 = ctx.now()
        saw_grasp = False
        loop_disabled = False
        while ctx.ok():
            if is_cancelled():
                pick_gate_off()
                return (False, "canceled during pick")
            st = ctx.vs_state
            elapsed = ctx.now() - t0
            feedback(f"PICK:{st}", min(0.55, 0.2 + 0.35 * elapsed / max(timeout, 1e-3)))
            if not loop_disabled and st not in ("", "DONE"):
                loop_disabled = ctx.set_bool_param("grasp_auto_loop", False)
            if st in PICK_ACTIVE_STATES:
                saw_grasp = True
            if not saw_grasp and elapsed > acquire_timeout:
                pick_gate_off()
                return (False, f"'{obj}' not acquired within {acquire_timeout:.0f}s "
                               "(not detected, or grasp pose out of workspace)")
            if saw_grasp and st == "DONE":
                ctx.set_bool_param("grasp_auto_loop", False)
                ctx.sleep(1.0)                        # let lift/width settle
                held = ctx.gripper_width > held_threshold
                # Leave grasp_enabled ON and the node at DONE so Phase 2 can re-arm
                # (DONE -> IDLE -> external GUARDED_APPROACH) while still holding.
                if held:
                    return (True, f"held: width={ctx.gripper_width:.4f}")
                pick_gate_off()
                return (False, f"likely empty grasp; width={ctx.gripper_width:.4f} "
                               f"<= held threshold {held_threshold:.4f}")
            if elapsed > timeout:
                pick_gate_off()
                return (False, f"timeout after {timeout:.0f}s in state '{st}'")
            ctx.sleep(0.1)
        pick_gate_off()
        return (False, "node shutting down during pick")

    def _blind_place(self, ctx, feedback, is_cancelled, grasp_xyz,
                     timeout) -> Tuple[bool, str]:
        """Place at the remembered target via the external-target servo path.

        In place mode the arm approaches the PREGRASP (standoff above the target)
        and releases there — it never descends to the grasp point. We therefore set
        the pregrasp to `place_clearance_m` directly above the captured destination
        point, decoupled from the (lower) pick standoff, so the held object clears
        the destination instead of bumping it. Tunable live:
        `ros2 param set /skill_server place_clearance_m <m>`.
        """
        place_clearance = float(ctx.get_param("place_clearance_m", 0.15))
        ext_pregrasp = [grasp_xyz[0], grasp_xyz[1], grasp_xyz[2] + place_clearance]
        ctx.log(f"[PnP] place clearance={place_clearance:.3f} m -> release approach "
                f"at z={ext_pregrasp[2]:.3f} (target z={grasp_xyz[2]:.3f})")
        # The high approach standoff (place_clearance_m) already lifts the held
        # object clear of the destination, so the node's extra pre-release hop is
        # redundant here and just looks like a jump. Zero place_release_clearance_m
        # for the place so the gripper opens right at the standoff, then restore the
        # node's value so the standalone `place` skill (lower standoff) keeps it.
        prev_clear = ctx.get_remote_params(
            ["place_release_clearance_m"]).get("place_release_clearance_m")
        ctx.set_double_param("place_release_clearance_m", 0.0)
        try:
            # Inject the remembered base-frame target, switch to place mode, and
            # re-arm (DONE -> IDLE) so handle_idle injects it into GUARDED_APPROACH.
            ctx.set_double_array_param("external_grasp_target", grasp_xyz)
            ctx.set_double_array_param("external_pregrasp_target", ext_pregrasp)
            ctx.set_bool_param("place_mode", True)
            ctx.set_bool_param("use_external_target", True)
            ctx.publish_prompt("")                    # detector dormant during place
            if not ctx.set_bool_param("grasp_enabled", True):
                return (False, "could not reach visual-servo node to start place")
            ctx.set_bool_param("grasp_auto_loop", True)   # bump DONE -> IDLE once

            t0 = ctx.now()
            saw_place = False
            loop_disabled = False
            while ctx.ok():
                if is_cancelled():
                    return (False, "canceled during place")
                st = ctx.vs_state
                elapsed = ctx.now() - t0
                feedback(f"PLACE:{st}",
                         min(0.95, 0.6 + 0.35 * elapsed / max(timeout, 1e-3)))
                if not loop_disabled and st not in ("", "DONE"):
                    loop_disabled = ctx.set_bool_param("grasp_auto_loop", False)
                if st in PLACE_ACTIVE_STATES:
                    saw_place = True
                if st == "LOST":
                    return (False, "place aborted (LOST) — target rejected by the "
                                   "node's reach/workspace guard")
                if saw_place and st == "DONE":
                    ctx.set_bool_param("grasp_auto_loop", False)
                    ctx.sleep(0.5)
                    return (True, f"released; width={ctx.gripper_width:.4f}")
                if elapsed > timeout:
                    return (False, f"timeout after {timeout:.0f}s in state '{st}'")
                ctx.sleep(0.1)
            return (False, "node shutting down during place")
        finally:
            if prev_clear is not None:
                ctx.set_double_param("place_release_clearance_m", float(prev_clear))


def _fmt(xyz: List[float]) -> str:
    return "(" + ", ".join(f"{v:.3f}" for v in xyz) + ")"


def _implausible(xyz: List[float]) -> Optional[str]:
    """Light sanity gate on a captured base-frame point. Returns a reason or None.

    The node's plane-fit + reach guards already vet the estimate; this only catches
    gross garbage (NaN/inf, or a point absurdly far from the arm base) before we
    commit to a pick.
    """
    import math
    if len(xyz) != 3 or not all(math.isfinite(v) for v in xyz):
        return "non-finite"
    x, y, z = xyz
    if math.sqrt(x * x + y * y + z * z) > 1.2:
        return "out of reach (>1.2 m from base)"
    return None
