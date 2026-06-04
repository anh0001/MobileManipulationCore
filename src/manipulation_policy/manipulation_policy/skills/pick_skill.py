# Copyright 2026 MobileManipulationCore Contributors
# Licensed under the Apache License, Version 2.0.
"""Pick skill — one-call "pick object X" over the visual-servo grasp pipeline.

Ported from the former pick_orchestrator node into the skill framework. The ROS
plumbing (prompt publish, remote params, arm move, state/width reads) now lives
in the skill_server; this file is just the grasp *sequence*.

Sequence per goal (mirrors scripts/grasp_autotune.py run_attempt/classify):
  1. publish the object label to the visual-servo prompt topic
  2. gate off, reset the arm to the look-down capture pose
  3. gate the grasp on (grasp_enabled=true, grasp_auto_loop=false -> single pick)
  4. wait for the state machine to enter the grasp sequence, then reach DONE
  5. classify object_held from the final gripper width, gate off, return

Held detection is heuristic (no force sensor): a jaw blocked by an object stops
short of full close, so width > held_width_threshold => held. Thin/soft objects
can still close fully while holding, so the raw final width is also returned.
"""
from __future__ import annotations

from typing import Any, Callable, Dict

from .base import Skill, SkillParam, SkillResult, SkillContext
from .registry import register_skill

# Active grasp-sequence states (object actually being picked); used to confirm a
# fresh attempt started and to detect a stale prior "DONE".
ACTIVE_STATES = {
    "OPEN_GRIPPER", "GUARDED_APPROACH", "APPROACH_DEPTH", "CLOSE_GRIPPER", "LIFT",
}


@register_skill
class PickSkill(Skill):
    name = "pick"
    description = (
        "Pick a single object by open-vocabulary name with the robot arm. "
        "Runs the full visual-servo grasp: detect -> align -> grasp -> lift. "
        "Returns success (reached grasp+lift), object_held (jaw blocked on "
        "something — heuristic, no force sensor), and gripper_width (m)."
    )
    params = [
        SkillParam("object", "string", required=True,
                   description='label of the thing to grasp, e.g. "bread", '
                               '"toy banana", "red apple". For toy fruits '
                               'prefix with "toy".'),
        SkillParam("timeout_sec", "number", default=0.0,
                   description="max seconds for the whole attempt; <=0 uses the "
                               "server default_timeout_sec."),
    ]

    def execute(self, ctx: SkillContext, params: Dict[str, Any],
                feedback: Callable[[str, float], None],
                is_cancelled: Callable[[], bool]) -> SkillResult:
        obj = str(params["object"]).strip()
        req_timeout = float(params["timeout_sec"])
        timeout = req_timeout if req_timeout > 0 else float(
            ctx.get_param("default_timeout_sec", 60.0))
        acquire_timeout = float(ctx.get_param("acquire_timeout_sec", 12.0))
        held_threshold = float(ctx.get_param("held_width_threshold", 0.012))
        reset_arm = bool(ctx.get_param("reset_arm_each_pick", True))
        capture_pose = list(ctx.get_param("capture_pose",
                                          [0.0, 1.2, -0.2, 0.0, -0.35, 0.0]))

        def gate_off():
            ctx.set_bool_param("grasp_enabled", False)
            ctx.set_bool_param("grasp_auto_loop", False)

        def done(success, held, message):
            gate_off()
            return SkillResult(success, message, {
                "object_held": bool(held),
                "gripper_width": round(float(ctx.gripper_width), 4),
            })

        if not obj:
            return SkillResult(False, "empty object label", {})
        ctx.log(f"[PICK '{obj}'] starting (timeout={timeout:.0f}s)")

        # 1. gate off, set the prompt. Enable auto_loop so a node sitting at a
        #    stale DONE re-arms (DONE -> IDLE -> ACQUIRE); turned back off the
        #    moment this pick starts, so exactly one pick runs.
        ctx.set_bool_param("grasp_enabled", False)
        ctx.set_bool_param("grasp_auto_loop", True)
        ctx.publish_prompt(obj)

        # 2. reset arm to the look-down capture pose
        if reset_arm:
            ctx.move_arm_to(capture_pose, time_sec=5.0)

        # 3. gate the grasp on
        feedback(ctx.vs_state, 0.0)
        if not ctx.set_bool_param("grasp_enabled", True):
            return done(False, False,
                        "could not reach visual-servo node to enable grasp")

        # 4. wait for the grasp sequence to start, then for DONE
        t0 = _mono(ctx)
        saw_grasp = False
        while ctx.ok():
            if is_cancelled():
                gate_off()
                return SkillResult(False, "canceled", {
                    "gripper_width": round(float(ctx.gripper_width), 4)})
            now = _mono(ctx)
            elapsed = now - t0
            feedback(ctx.vs_state, min(0.95, elapsed / timeout) if timeout else 0.0)
            if ctx.vs_state in ACTIVE_STATES:
                if not saw_grasp:
                    # pick started — stop auto_loop so it won't re-fire after DONE
                    ctx.set_bool_param("grasp_auto_loop", False)
                saw_grasp = True
            if not saw_grasp and elapsed > acquire_timeout:
                return done(False, False,
                            f"'{obj}' not acquired within {acquire_timeout:.0f}s "
                            "(not detected, or grasp pose out of workspace)")
            if saw_grasp and ctx.vs_state == "DONE":
                ctx.sleep(1.0)  # let the lift/width settle
                held = ctx.gripper_width > held_threshold
                msg = (f"picked '{obj}'; width={ctx.gripper_width:.4f} "
                       f"({'held (jaw blocked)' if held else 'full close — held only if thin/soft'})")
                return done(True, held, msg)
            if elapsed > timeout:
                return done(False, False,
                            f"timeout after {timeout:.0f}s in state '{ctx.vs_state}' "
                            "(stuck — e.g. grasp target rejected by workspace gate)")
            ctx.sleep(0.1)
        return done(False, False, "node shutting down")


def _mono(ctx: SkillContext) -> float:
    return ctx.now()
