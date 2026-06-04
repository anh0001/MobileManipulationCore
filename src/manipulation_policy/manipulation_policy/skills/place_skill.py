# Copyright 2026 MobileManipulationCore Contributors
# Licensed under the Apache License, Version 2.0.
"""Place skill — put the held object into/onto a detected target (a box, a table).

Mirror of the pick skill, but for releasing. Run after a successful `pick` (the
arm is holding an object). The skill names a target to DETECT (e.g. "box",
"table"), drives the same visual-servo pipeline that pick uses, and — in
place_mode — approaches the standoff ABOVE the detected target and opens the
gripper there to release, instead of descending and closing.

Sequence per goal:
  1. publish the target label to the visual-servo prompt topic
  2. gate off, reset the arm to the look-down capture pose (to see the target)
  3. set place_mode=true and gate the grasp on
  4. wait for the state machine to align, approach, OPEN_GRIPPER (release), LIFT
  5. reach DONE, gate off + clear place_mode, return

The release happens at pregrasp_standoff_m above the target surface (no descent
onto it), so the object drops into the box / onto the table.
"""
from __future__ import annotations

from typing import Any, Callable, Dict

from .base import Skill, SkillParam, SkillResult, SkillContext
from .registry import register_skill

# States that confirm the place sequence is actually running.
ACTIVE_STATES = {
    "ALIGN_XY", "ESTIMATE_GRASP", "GUARDED_APPROACH", "OPEN_GRIPPER", "LIFT",
}


@register_skill
class PlaceSkill(Skill):
    name = "place"
    description = (
        "Place the currently-held object into/onto a detected target. Names a "
        "receptacle to detect (e.g. \"box\", \"table\"), runs the visual-servo "
        "pipeline to localize it, approaches the standoff above it, and opens "
        "the gripper to release. Run after a successful pick. Returns success "
        "and the final gripper width (should be open)."
    )
    params = [
        SkillParam("target", "string", required=True,
                   description='open-vocabulary label of where to place, e.g. '
                               '"box", "table", "bowl", "plate".'),
        SkillParam("timeout_sec", "number", default=0.0,
                   description="max seconds for the whole attempt; <=0 uses the "
                               "server default_timeout_sec."),
    ]

    def execute(self, ctx: SkillContext, params: Dict[str, Any],
                feedback: Callable[[str, float], None],
                is_cancelled: Callable[[], bool]) -> SkillResult:
        target = str(params["target"]).strip()
        req_timeout = float(params["timeout_sec"])
        timeout = req_timeout if req_timeout > 0 else float(
            ctx.get_param("default_timeout_sec", 60.0))
        acquire_timeout = float(ctx.get_param("acquire_timeout_sec", 12.0))
        reset_arm = bool(ctx.get_param("reset_arm_each_pick", True))
        capture_pose = list(ctx.get_param("capture_pose",
                                          [0.0, 1.2, -0.2, 0.0, -0.35, 0.0]))

        def gate_off():
            ctx.set_bool_param("grasp_enabled", False)
            ctx.set_bool_param("grasp_auto_loop", False)
            ctx.set_bool_param("place_mode", False)  # leave the node ready to pick

        def done(success, message):
            gate_off()
            return SkillResult(success, message, {
                "gripper_width": round(float(ctx.gripper_width), 4)})

        if not target:
            return SkillResult(False, "empty target label", {})
        ctx.log(f"[PLACE -> '{target}'] starting (timeout={timeout:.0f}s)")

        # 1. gate off, set the target prompt, arm place_mode + re-arm via auto_loop
        ctx.set_bool_param("grasp_enabled", False)
        ctx.set_bool_param("place_mode", True)
        ctx.set_bool_param("grasp_auto_loop", True)
        ctx.publish_prompt(target)

        # 2. look down so the target is in view
        if reset_arm:
            ctx.move_arm_to(capture_pose, time_sec=5.0)

        # 3. gate on
        feedback(ctx.vs_state, 0.0)
        if not ctx.set_bool_param("grasp_enabled", True):
            return done(False, "could not reach visual-servo node to start place")

        # 4. wait for the place sequence to start, then for DONE
        t0 = ctx.now()
        saw_seq = False
        while ctx.ok():
            if is_cancelled():
                return done(False, "canceled")
            elapsed = ctx.now() - t0
            feedback(ctx.vs_state, min(0.95, elapsed / timeout) if timeout else 0.0)
            if ctx.vs_state in ACTIVE_STATES:
                if not saw_seq:
                    ctx.set_bool_param("grasp_auto_loop", False)  # one place only
                saw_seq = True
            if not saw_seq and elapsed > acquire_timeout:
                return done(False,
                            f"'{target}' not acquired within {acquire_timeout:.0f}s "
                            "(not detected, or out of workspace)")
            if saw_seq and ctx.vs_state == "DONE":
                ctx.sleep(0.5)
                width = round(float(ctx.gripper_width), 4)
                return done(True, f"released object on '{target}'; width={width}")
            if elapsed > timeout:
                return done(False,
                            f"timeout after {timeout:.0f}s in state '{ctx.vs_state}'")
            ctx.sleep(0.1)
        return done(False, "node shutting down")
