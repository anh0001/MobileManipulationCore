#!/usr/bin/env python3
# Copyright 2026 MobileManipulationCore Contributors
# Licensed under the Apache License, Version 2.0.
"""Autonomous closed-loop tuner for the look-then-move table-plane grasp.

Deterministic ROS 2 supervisor (no human in the loop): for each attempt it
resets the arm to the capture pose, sets the tuning params on the running
visual_servo_node via `ros2 param set` (the node re-reads them in IDLE), gates a
single grasp attempt with `grasp_enabled`, records an MCAP bag, watches the
state machine + gripper joint7 to classify the outcome, then updates the bounded
params (coordinate/pattern descent) and repeats until it grasps N times or hits a
hard-abort / escalation condition.

Prereqs (run these first):
  1. robot bringup running (pin the D405: wrist_camera_serial:=<D405 serial>)
  2. local Grounding DINO server up (VSCode task: "Run Grounding DINO Server (Local Jetson Docker)")
  3. grasp pipeline running in auto-loop mode, e.g.:
       PYTHONNOUSERSITE=1 ros2 launch manipulation_bringup core_launch.py \
         control_mode:=visual_servo arm_execution_mode:=move_group \
         visual_servo_grasp_auto_loop:=true   # or `ros2 param set /visual_servo_node grasp_auto_loop true`

Then:  PYTHONNOUSERSITE=1 python3 scripts/grasp_autotune.py

Success detector (wrist cam + joint7 only, no force sensor):
  primary  : joint7 settles NONZERO and stable after lift (object between fingers)
  secondary: object no longer detected in the original table ROI after lift
  negative : object still detected on the table  -> failed
Outcome is logged per attempt to ~/grasp_autotune.jsonl and an MCAP bag per
attempt under ~/grasp_autotune_bags/.
"""
import json
import math
import os
import signal
import subprocess
import time
from dataclasses import dataclass, field, asdict

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterValue, ParameterType

NODE = "/visual_servo_node"
ARM_FJT = "/piper_arm_controller/follow_joint_trajectory"
ARM_JOINTS = [f"piper_joint{i}" for i in range(1, 7)]
CAPTURE_POSE = [0.0, 1.2, -0.2, 0.0, -0.35, 0.0]  # look-down

# joint7 (gripper) thresholds — calibrated: open ~0.069, closed 0.0
J7_OPEN = 0.045      # considered "open enough" to straddle the object
J7_HELD = 0.020      # after lift, > this width => object between fingers
J7_EMPTY = 0.010     # near fully closed

# Bounded param search (base frame, meters). Coordinate/pattern descent.
PARAMS = {
    "grasp_offset_x": dict(val=-0.035, lo=-0.10, hi=0.05, step=0.01),
    "grasp_offset_y": dict(val=-0.018, lo=-0.10, hi=0.06, step=0.01),
    "grasp_height_above_table_m": dict(val=0.055, lo=0.03, hi=0.12, step=0.005),
}

BAG_DIR = os.path.expanduser("~/grasp_autotune_bags")
RESULT_LOG = os.path.expanduser("~/grasp_autotune.jsonl")
# Light topic set — recording the raw color/depth image streams loads the Jetson
# enough to starve the visual_servo RGB callback (ACQUIRE then times out). These
# are sufficient for the joint7/state/detection classifier; set RECORD_IMAGES=1
# to add the heavy image topics for offline debugging on a less-loaded machine.
RECORD_TOPICS = [
    "/piper/joint_states", "/arm_status", "/visual_servo/state",
    "/manipulation/target_detections",
    "/piper/wrist_camera/piper_d405/color/camera_info",
    "/tf", "/tf_static", "/rosout",
]
if os.environ.get("RECORD_IMAGES") == "1":
    RECORD_TOPICS += [
        "/piper/wrist_camera/piper_d405/color/image_raw",
        "/piper/wrist_camera/piper_d405/depth/image_rect_raw",
    ]
SUCCESS_TARGET = 3        # consecutive successes to declare done
ATTEMPT_TIMEOUT = 60.0    # s, abort an attempt that never reaches DONE
MAX_ATTEMPTS = 40
ESCALATE_REPEAT = 3       # same code-failure mode N times in a row -> stop


@dataclass
class Attempt:
    idx: int
    params: dict
    states: list = field(default_factory=list)        # (t, state)
    j7: list = field(default_factory=list)            # (t, value)
    det_before: int = 0                               # detections on table before
    det_after: int = 0                                # detections on table after lift
    outcome: str = "unknown"
    j7_peak: float = 0.0
    j7_final: float = 0.0
    note: str = ""


class AutoTuner(Node):
    def __init__(self):
        super().__init__("grasp_autotuner")
        self.j7 = 0.0
        self.j7_hist = []          # rolling (t, value)
        self.state = ""
        self.state_hist = []       # (t, state)
        self.det_count = 0
        self.err_code = 0
        self.create_subscription(JointState, "/piper/joint_states", self._js, qos_profile_sensor_data)
        self.create_subscription(String, "/visual_servo/state", self._state, 10)
        self.create_subscription(Detection2DArray, "/manipulation/target_detections", self._det, qos_profile_sensor_data)
        # arm_status err_code is a hard-abort signal; optional (needs piper_msgs).
        try:
            from piper_msgs.msg import PiperStatusMsg
            self.create_subscription(PiperStatusMsg, "/arm_status", self._arm_status, 10)
        except Exception:
            self.get_logger().warn("piper_msgs PiperStatusMsg unavailable; err_code abort disabled")
        self.arm = ActionClient(self, FollowJointTrajectory, ARM_FJT)
        self.param_cli = self.create_client(SetParameters, NODE + "/set_parameters")
        os.makedirs(BAG_DIR, exist_ok=True)

    # --- callbacks ---
    def _js(self, m):
        try:
            i = m.name.index("piper_joint7")
            self.j7 = m.position[i]
            self.j7_hist.append((time.time(), self.j7))
            if len(self.j7_hist) > 4000:
                self.j7_hist = self.j7_hist[-2000:]
        except (ValueError, IndexError):
            pass

    def _state(self, m):
        if m.data != self.state:
            self.state = m.data
            self.state_hist.append((time.time(), m.data))

    def _det(self, m):
        self.det_count = len(m.detections)

    def _arm_status(self, m):
        try:
            self.err_code = int(m.err_code)
        except Exception:
            pass

    def spin_for(self, dur):
        end = time.time() + dur
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    # --- robot control ---
    def set_param(self, name, value):
        # Use the SetParameters service (the `ros2 param set` CLI mangles negative
        # numeric values, treating the leading '-' as an option).
        if not self.param_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f"set_parameters service unavailable for {name}")
            return False
        pv = ParameterValue()
        if isinstance(value, bool):
            pv.type = ParameterType.PARAMETER_BOOL
            pv.bool_value = value
        elif isinstance(value, float):
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = float(value)
        elif isinstance(value, int):
            pv.type = ParameterType.PARAMETER_INTEGER
            pv.integer_value = value
        else:
            pv.type = ParameterType.PARAMETER_STRING
            pv.string_value = str(value)
        req = SetParameters.Request(parameters=[ParamMsg(name=name, value=pv)])
        fut = self.param_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.result() is not None

    def move_to_capture(self):
        if not self.arm.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("arm action server unavailable")
            return False
        g = FollowJointTrajectory.Goal()
        g.trajectory = JointTrajectory(joint_names=ARM_JOINTS)
        p = JointTrajectoryPoint()
        p.positions = CAPTURE_POSE
        p.time_from_start.sec = 5
        g.trajectory.points = [p]
        fut = self.arm.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
        gh = fut.result()
        if not gh or not gh.accepted:
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=10.0)
        return True

    def start_bag(self, idx):
        path = os.path.join(BAG_DIR, f"attempt_{idx:03d}")
        proc = subprocess.Popen(
            ["ros2", "bag", "record", "-s", "mcap", "-o", path, "--include-hidden-topics"] + RECORD_TOPICS,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        return proc

    def stop_bag(self, proc):
        if proc and proc.poll() is None:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)

    # --- one attempt ---
    def run_attempt(self, idx, params):
        a = Attempt(idx=idx, params=dict(params))
        # 1. gate off, reset arm, apply params
        self.set_param("grasp_enabled", False)
        self.spin_for(0.5)
        self.move_to_capture()
        self.spin_for(1.0)
        for k, v in params.items():
            self.set_param(k, float(v))
        # baseline: object should be detected on the table
        self.spin_for(2.0)
        a.det_before = self.det_count
        # 2. record + gate on
        bag = self.start_bag(idx)
        self.state_hist = []
        self.j7_hist = []
        self.set_param("grasp_enabled", True)
        # 3. observe a FRESH grasp cycle: wait for the grasp to actually start
        # (OPEN/approach/close), then for DONE. A stale "DONE" from a prior cycle
        # must not be mistaken for this attempt's completion.
        t0 = time.time()
        reached_done = False
        saw_grasp = False
        active = {"OPEN_GRIPPER", "GUARDED_APPROACH", "APPROACH_DEPTH", "CLOSE_GRIPPER", "LIFT"}
        while time.time() - t0 < ATTEMPT_TIMEOUT and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.state in active:
                saw_grasp = True
            if saw_grasp and self.state == "DONE":
                self.spin_for(1.5)
                reached_done = True
                break
        self.set_param("grasp_enabled", False)
        a.det_after = self.det_count
        self.spin_for(0.5)
        self.stop_bag(bag)
        # 4. fill timeline + classify
        a.states = [(round(t - t0, 2), s) for (t, s) in self.state_hist]
        a.j7 = [(round(t - t0, 2), round(v, 4)) for (t, v) in self.j7_hist if t >= t0]
        a.outcome, a.j7_peak, a.j7_final, a.note = self.classify(a, reached_done, saw_grasp)
        return a

    def classify(self, a, reached_done, saw_grasp=True):
        if not saw_grasp:
            return "no_grasp_started", 0.0, 0.0, "node never entered the grasp sequence (detection/gate?)"
        j7 = [v for _, v in a.j7]
        peak = max(j7) if j7 else 0.0
        # state phase windows
        def t_of(state):
            for t, s in a.states:
                if s == state:
                    return t
            return None
        t_guarded = t_of("GUARDED_APPROACH")
        t_close = t_of("CLOSE_GRIPPER")
        t_lift = t_of("LIFT")
        # j7 during the descent (GUARDED before CLOSE)
        desc = [v for (t, v) in a.j7 if t_guarded is not None and t_close is not None
                and t_guarded <= t < t_close]
        desc_open = (max(desc) if desc else 0.0)
        # j7 after lift (final settled)
        after = [v for (t, v) in a.j7 if t_lift is not None and t >= t_lift]
        final = (sum(after[-10:]) / len(after[-10:])) if len(after) >= 1 else (j7[-1] if j7 else 0.0)

        if self.err_code != 0:
            return "abort_err", peak, final, f"arm err_code={self.err_code}"
        if not reached_done:
            return "abort_timeout", peak, final, "never reached DONE"
        if peak < J7_OPEN:
            return "gripper_did_not_open", peak, final, f"peak j7={peak:.3f} < {J7_OPEN}"
        if desc and desc_open < J7_OPEN:
            return "gripper_closed_early", peak, final, "gripper closed before/at descent"
        # reached close with gripper having been open
        if final > J7_HELD and a.det_after < a.det_before:
            return "success", peak, final, f"held width={final:.3f}, object gone from table"
        if final > J7_HELD:
            return "success_uncertain", peak, final, f"held width={final:.3f} but object still detected"
        # empty close
        if a.det_after >= max(1, a.det_before):
            return "miss_empty", peak, final, "empty close, object still on table"
        return "unknown", peak, final, "no rule matched"

    # --- param update: pattern (coordinate) descent over xy, then height ---
    def update_params(self, params, outcome, history):
        # Only adjust geometry for empty misses. Code-level failures escalate.
        if outcome not in ("miss_empty", "unknown", "success_uncertain"):
            return params, False
        # simple expanding spiral over (offset_x, offset_y); height nudged if many misses
        seq = [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1),
               (2, 0), (-2, 0), (0, 2), (0, -2)]
        n_miss = sum(1 for h in history if h.startswith("miss") or h == "unknown")
        dx, dy = seq[min(n_miss, len(seq) - 1)]
        new = dict(params)
        sx = PARAMS["grasp_offset_x"]["step"]
        sy = PARAMS["grasp_offset_y"]["step"]
        new["grasp_offset_x"] = clamp(PARAMS["grasp_offset_x"]["val"] + dx * sx,
                                      PARAMS["grasp_offset_x"]["lo"], PARAMS["grasp_offset_x"]["hi"])
        new["grasp_offset_y"] = clamp(PARAMS["grasp_offset_y"]["val"] + dy * sy,
                                      PARAMS["grasp_offset_y"]["lo"], PARAMS["grasp_offset_y"]["hi"])
        return new, True


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main():
    rclpy.init()
    tuner = AutoTuner()
    params = {k: PARAMS[k]["val"] for k in PARAMS}
    successes = 0
    history = []        # outcome strings
    log = open(RESULT_LOG, "a")
    tuner.get_logger().info("autotune starting; ensure bringup + DINO + grasp pipeline are running")
    # Put the node in auto-loop mode (DONE -> IDLE) and start gated off.
    tuner.set_param("grasp_auto_loop", True)
    tuner.set_param("grasp_enabled", False)
    tuner.spin_for(1.0)
    try:
        for idx in range(MAX_ATTEMPTS):
            a = tuner.run_attempt(idx, params)
            history.append(a.outcome)
            rec = asdict(a)
            rec["stamp"] = time.time()
            log.write(json.dumps(rec) + "\n")
            log.flush()
            tuner.get_logger().info(
                f"[#{idx}] outcome={a.outcome} j7_peak={a.j7_peak:.3f} j7_final={a.j7_final:.3f} "
                f"det {a.det_before}->{a.det_after} | {a.note}")
            if a.outcome in ("success",):
                successes += 1
                if successes >= SUCCESS_TARGET:
                    tuner.get_logger().info(f"DONE: {successes} successes. params={params}")
                    break
                continue
            successes = 0
            # escalate on repeated code-level failure
            recent = history[-ESCALATE_REPEAT:]
            if len(recent) >= ESCALATE_REPEAT and len(set(recent)) == 1 and \
               recent[0] in ("gripper_did_not_open", "gripper_closed_early", "abort_err",
                             "abort_timeout", "no_grasp_started"):
                tuner.get_logger().error(
                    f"ESCALATE: '{recent[0]}' x{ESCALATE_REPEAT} — not a param issue, needs a human/code fix. Stopping.")
                break
            params, changed = tuner.update_params(params, a.outcome, history)
            if changed:
                tuner.get_logger().info(f"updated params -> offset=({params['grasp_offset_x']:.3f},"
                                        f"{params['grasp_offset_y']:.3f}) h={params['grasp_height_above_table_m']:.3f}")
    except KeyboardInterrupt:
        pass
    finally:
        tuner.set_param("grasp_enabled", False)
        log.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
