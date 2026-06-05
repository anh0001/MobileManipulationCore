# Copyright 2026 MobileManipulationCore Contributors
# Licensed under the Apache License, Version 2.0.
"""Handover skill — present a grasped object to a person and release it.

One-call "hand this to the person". Uses the base-mounted RealSense **D435i** to
localize a person, then presents the held object toward them at a safe distance,
fast and smooth, and opens the gripper so they can take it.

Why the D435i is grabbed on demand (not a ROS stream)
-----------------------------------------------------
The D435i is a free USB device — only the wrist D405 runs as a realsense ROS
node — so streaming it continuously over ROS just to take one snapshot is pure
overhead. Instead this skill opens the D435i by serial with pyrealsense2, grabs a
single aligned color+depth frame on demand, then closes it. No new node, no new
topic, no continuous bandwidth. (Conflict-free: nothing else owns the D435i.)

Pipeline per goal
-----------------
  1. Pre-check: an object must be held (gripper not fully closed).
  2. Capture: one aligned color+depth frame from the D435i (warm up a few frames
     for auto-exposure, then stop the pipeline).
  3. Detect: POST the color frame to the existing Grounding DINO HTTP service with
     prompt "person" (open-vocabulary, already deployed — no new model). Select
     the dominant, central person; abort if two people are equally plausible.
  4. Localize: sample a median depth over an upper-torso ROI, deproject to a 3D
     point in the D435i optical frame, map it to the arm-base XY plane with a
     horizontal extrinsic (camera position + yaw vs the arm base).
  5. Gate: the person must be within a distance band and an azimuth sector;
     otherwise abort (e.g. too close => ask them to step back).
  6. Present: yaw the arm toward the person (joint1 = azimuth) via a taught,
     compact *staging* pose, then extend a taught *present* pose. Two short moves
     so the held object is never swept sideways at full extension.
  7. Release: dwell so the person can grasp it, then open the gripper and retract
     to the ready pose.

CALIBRATION REQUIRED before first real use (see config/handover_params.yaml)
----------------------------------------------------------------------------
  * ``handover_camera_x_m/_y_m/_yaw_rad`` — where the D435i is mounted relative to
    the arm base (it is *behind-and-right* of the arm). The defaults are a rough
    guess; MEASURE/tune them or the chosen direction will be biased.
  * ``handover_staging_pose`` / ``handover_present_pose`` — hand-teach these on the
    real arm. They DEFAULT to the safe look-down ready pose, so an untaught run
    only yaws and holds (it will not fling the arm); teach a real forward-reach
    ``present_pose`` to actually extend the object toward the person.

Design choices (reviewed): a horizontal extrinsic (not an uncalibrated full 6-DOF
pose) is enough to choose a safe *direction*; taught joint poses (not the
visual-servo "place" primitive, which drops from above with a downward gripper)
suit a forward handover; and a timed dwell (no force sensor) is the only reliable
release default.
"""
from __future__ import annotations

import base64
import json
import math
from typing import Any, Callable, Dict, List, Optional, Tuple
from urllib import error as urlerror
from urllib import request as urlrequest

import numpy as np

from .base import Skill, SkillParam, SkillResult, SkillContext
from .registry import register_skill

# Server tunables (declared by skill_server via Skill.server_params; overridable
# from config/handover_params.yaml and at runtime via `ros2 param set`). Floats
# are written as x.0 so the ROS parameter type is DOUBLE — keep config overrides
# type-consistent.
HANDOVER_PARAM_DEFAULTS: Dict[str, Any] = {
    # --- D435i capture (pyrealsense2, on demand) ---
    # Extra site-packages to expose pyrealsense2/cv2 to the skill_server process.
    # On Jetson the ROS env runs PYTHONNOUSERSITE=1 (numpy pinned to the ROS build)
    # so the user-site pyrealsense2/cv2 aren't importable; this path is appended at
    # runtime AFTER numpy is loaded (so the loaded numpy is unchanged). "" = import
    # normally (correct when the deps are already on the ROS path).
    "handover_extra_site_packages": "",
    # "" -> first connected RealSense (use only when the D435i is the sole device)
    "handover_camera_serial": "243722070013",
    "handover_color_width": 640,
    "handover_color_height": 480,
    "handover_fps": 30,
    "handover_warmup_frames": 12,               # drop these so auto-exposure settles
    "handover_capture_timeout_sec": 5.0,
    # The D435i can drop into a no-frames state under rapid open/close cycling.
    # On a capture error: re-open once cheaply, then hardware-reset + retry.
    "handover_capture_retries": 2,
    "handover_capture_reset_on_fail": True,
    "handover_capture_reset_wait_sec": 7.0,
    # --- person detection (reuses the Grounding DINO HTTP service) ---
    "handover_detect_url": "http://localhost:30543/detect",
    "handover_detect_prompt": "person",
    "handover_detect_timeout_sec": 8.0,
    "handover_score_min": 0.35,
    "handover_min_box_height_frac": 0.20,       # reject boxes shorter than this frac of image
    "handover_ambiguous_area_ratio": 0.75,      # 2nd box ~as big AND far apart -> abort
    # --- depth sampling (upper-torso point) ---
    "handover_torso_frac": 0.40,                # depth at y_top + torso_frac*box_height
    "handover_depth_roi_half_px": 10,
    "handover_min_depth_pixels": 25,
    "handover_min_valid_depth_m": 0.20,
    # --- D435i extrinsic vs arm base (piper_base_link) — CALIBRATE ---
    # Optical frame: x=right, y=down, z=forward. The horizontal forward = z and y
    # are first leveled by camera_roll (the up/down tilt), then forward + left(=-x)
    # are rotated by camera_yaw (CCW+ about base +Z) and translated by (x, y).
    "handover_camera_x_m": -0.12,               # behind the arm base (-X)
    "handover_camera_y_m": -0.18,               # to the right of the arm base (-Y)
    "handover_camera_yaw_rad": 0.0,             # camera forward vs base +X
    "handover_camera_roll_rad": 0.0,            # camera up/down tilt: + look up, - look down
    # --- safety gates ---
    "handover_min_distance_m": 0.75,            # abort if the person is closer (keep a safe gap)
    "handover_max_distance_m": 2.0,             # abort if farther (out of handover range)
    "handover_max_azimuth_rad": 1.0,            # abort if too far to the side (~57 deg)
    "handover_joint1_limit_rad": 2.5,           # clamp joint1 (arm limit is 2.618)
    "handover_require_object": True,            # require a held object before handing over
    # --- motion / release (TEACH the poses; defaults = safe ready pose) ---
    "handover_staging_pose": [0.0, 1.2, -0.2, 0.0, -0.35, 0.0],
    # present height is ADAPTIVE: the arm presents at a joint-space interpolation
    # between the LOW (present_pose) and HIGH (present_pose_standing) taught poses,
    # blended by the person's HEAD height between the two reference heights below.
    # Short person/seated -> near LOW; tall person/standing -> near HIGH.
    "handover_present_pose": [0.0, 1.2, -0.2, 0.0, -0.35, 0.0],           # LOW pose
    "handover_present_pose_standing": [0.0, 1.2, -0.2, 0.0, -0.35, 0.0],  # HIGH (teach higher)
    # Stature is estimated at the HEAD (this fraction down from the box top), using
    # the torso depth -> a far stronger standing/tall signal than a mid-torso point,
    # and a cut-off head (close, tall person) reads tall. 0 = box top.
    "handover_height_sample_frac": 0.05,
    # Person (head) height (metres above the camera) mapped to the LOW and HIGH
    # poses. Read the result's person_height_m for a seated and a standing person
    # to set these; below low -> full LOW, above high -> full HIGH, linear between.
    "handover_person_height_low_m": 0.0,
    "handover_person_height_high_m": 0.5,
    "handover_stage_time_sec": 2.5,
    "handover_present_time_sec": 3.0,
    "handover_dwell_sec": 2.5,                  # let the person take the object before opening
    "handover_retract_time_sec": 3.0,
}


@register_skill
class HandoverSkill(Skill):
    name = "handover"
    description = (
        "Hand the currently-held object to a person. Uses the base-mounted D435i "
        "camera to detect the nearest person and their distance, yaws the arm "
        "toward them, presents the object at a safe distance, dwells so they can "
        "take it, then opens the gripper. Run after a successful pick. Aborts if "
        "no object is held, no clear single person is seen, or the person is too "
        "close / too far / too far to the side."
    )
    params = [
        SkillParam("dwell_sec", "number", default=0.0,
                   description="seconds to hold the object out before opening the "
                               "gripper; <=0 uses the server handover_dwell_sec."),
        SkillParam("posture", "string", default="auto",
                   description='recipient posture, sets the present height: "auto" '
                               '(default) picks it from the detected head height; '
                               '"standing" forces the high pose, "seated" the low '
                               'pose. Use standing/seated when you know it — the '
                               'camera can be unsure up close.'),
        SkillParam("timeout_sec", "number", default=0.0,
                   description="reserved for future use; the handover is bounded by "
                               "its capture/detect/move timeouts."),
    ]
    server_params = HANDOVER_PARAM_DEFAULTS

    def execute(self, ctx: SkillContext, params: Dict[str, Any],
                feedback: Callable[[str, float], None],
                is_cancelled: Callable[[], bool]) -> SkillResult:
        # ---- read tunables -------------------------------------------------
        g = lambda n: ctx.get_param(n, HANDOVER_PARAM_DEFAULTS[n])  # noqa: E731
        require_object = bool(g("handover_require_object"))
        held_threshold = float(ctx.get_param("held_width_threshold", 0.004))
        dwell = float(params.get("dwell_sec") or 0.0) or float(g("handover_dwell_sec"))
        capture_pose = list(ctx.get_param("capture_pose",
                                          [0.0, 1.2, -0.2, 0.0, -0.35, 0.0]))

        result_data: Dict[str, Any] = {
            "gripper_width": round(float(ctx.gripper_width), 4),
            "released": False,
        }

        def fail(message: str) -> SkillResult:
            self._quiet_gate_off(ctx)
            result_data["gripper_width"] = round(float(ctx.gripper_width), 4)
            return SkillResult(False, message, result_data)

        # ---- 0. preconditions ---------------------------------------------
        feedback("CHECK", 0.0)
        if require_object and ctx.gripper_width <= held_threshold:
            return fail(f"no object held (gripper width {ctx.gripper_width:.4f} "
                        f"<= {held_threshold:.4f}); pick something first")
        # Make sure the visual servo isn't driving the arm while we take over.
        self._quiet_gate_off(ctx)

        # ---- 1. capture one D435i frame -----------------------------------
        feedback("CAPTURE", 0.1)
        extra_site = str(g("handover_extra_site_packages"))
        try:
            captured = self._capture_frame(
                serial=str(g("handover_camera_serial")),
                width=int(g("handover_color_width")),
                height=int(g("handover_color_height")),
                fps=int(g("handover_fps")),
                warmup=int(g("handover_warmup_frames")),
                timeout_sec=float(g("handover_capture_timeout_sec")),
                extra_site=extra_site,
                retries=int(g("handover_capture_retries")),
                reset_on_fail=bool(g("handover_capture_reset_on_fail")),
                reset_wait_sec=float(g("handover_capture_reset_wait_sec")))
        except Exception as e:  # pyrealsense errors, device busy, no device
            return fail(f"D435i capture failed: {e}")
        if captured is None:
            return fail("D435i returned no aligned color+depth frame")
        color_bgr, depth_m, intr = captured
        if is_cancelled():
            return fail("canceled after capture")

        # ---- 2. detect a person -------------------------------------------
        feedback("DETECT", 0.25)
        try:
            dets, img_w, img_h = self._detect_person(
                color_bgr,
                url=str(g("handover_detect_url")),
                prompt=str(g("handover_detect_prompt")),
                timeout=float(g("handover_detect_timeout_sec")),
                extra_site=extra_site)
        except Exception as e:
            return fail(f"person detection failed: {e}")
        img_w = int(img_w or color_bgr.shape[1])
        img_h = int(img_h or color_bgr.shape[0])
        people = _select_people(dets, img_w, img_h,
                                score_min=float(g("handover_score_min")),
                                min_box_height_frac=float(g("handover_min_box_height_frac")))
        result_data["num_people"] = len(people)
        if not people:
            return fail("no person detected in the D435i view")
        amb = _ambiguous(people, img_w, ratio=float(g("handover_ambiguous_area_ratio")))
        if amb:
            return fail("two people are equally plausible; cannot choose a "
                        "recipient (ask one to step forward)")

        # ---- 3. localize: torso depth -> 3D -> arm-base azimuth/distance ---
        feedback("LOCALIZE", 0.35)
        torso_frac = float(g("handover_torso_frac"))
        roi_half = int(g("handover_depth_roi_half_px"))
        min_px = int(g("handover_min_depth_pixels"))
        min_depth = float(g("handover_min_valid_depth_m"))
        height_frac = float(g("handover_height_sample_frac"))
        cam_x = float(g("handover_camera_x_m"))
        cam_y = float(g("handover_camera_y_m"))
        cam_yaw = float(g("handover_camera_yaw_rad"))
        cam_roll = float(g("handover_camera_roll_rad"))

        chosen = None
        for person in people:                       # largest (nearest) first; skip bad depth
            tx, ty = _torso_pixel(person, torso_frac)
            z = _depth_median_roi(depth_m, tx, ty, roi_half, min_px, min_depth)
            if z is None:
                continue
            x_opt, y_opt, z_opt = _deproject(tx, ty, z, intr)   # optical: x=right,y=down,z=fwd
            forward_h = _horizontal_forward(z_opt, y_opt, cam_roll)  # level the up/down tilt
            px, py = _optical_to_base(forward=forward_h, left=-x_opt,
                                      cam_x=cam_x, cam_y=cam_y, cam_yaw=cam_yaw)
            # Stature from the HEAD: deproject the box-top pixel at the torso depth.
            head_py = (person["cy"] - person["h"] * 0.5) + height_frac * person["h"]
            _, head_y_opt, _ = _deproject(person["cx"], head_py, z, intr)
            chosen = {
                "person": person, "depth_m": z,
                "base_xy": (px, py),
                "azimuth_rad": math.atan2(py, px),
                "distance_m": math.hypot(px, py),
                # head height above the camera (leveled) -> standing/tall vs seated
                "person_height_m": _height_above_camera(z, head_y_opt, cam_roll),
            }
            break
        if chosen is None:
            return fail("person detected but no valid depth on the torso "
                        "(out of the D435i depth range, or occluded)")

        az = chosen["azimuth_rad"]
        dist = chosen["distance_m"]
        result_data.update({
            "person_score": round(float(chosen["person"]["score"]), 3),
            "depth_m": round(float(chosen["depth_m"]), 3),
            "person_base_xy": [round(chosen["base_xy"][0], 3), round(chosen["base_xy"][1], 3)],
            "azimuth_rad": round(az, 4),
            "distance_m": round(dist, 3),
        })
        ctx.log(f"[HANDOVER] person at dist={dist:.2f}m azimuth={math.degrees(az):.0f}deg "
                f"(base xy={chosen['base_xy'][0]:.2f},{chosen['base_xy'][1]:.2f}) "
                f"depth={chosen['depth_m']:.2f}m score={chosen['person']['score']:.2f}")

        # ---- 4. safety gates ----------------------------------------------
        min_d = float(g("handover_min_distance_m"))
        max_d = float(g("handover_max_distance_m"))
        max_az = float(g("handover_max_azimuth_rad"))
        if dist < min_d:
            return fail(f"person too close ({dist:.2f}m < {min_d:.2f}m); ask them "
                        "to step back before the handover")
        if dist > max_d:
            return fail(f"person too far ({dist:.2f}m > {max_d:.2f}m) for a handover")
        if abs(az) > max_az:
            return fail(f"person too far to the side ({math.degrees(az):.0f}deg > "
                        f"{math.degrees(max_az):.0f}deg); reorient the base first")
        if is_cancelled():
            return fail("canceled before presenting")

        # ---- 5. present: yaw toward the person, then extend ---------------
        # Present height is ADAPTIVE: interpolate joint-space between the LOW
        # (present_pose) and HIGH (present_pose_standing) taught poses by the
        # person's torso height, so a taller/standing person is presented higher
        # and a shorter/seated one lower — continuously, not a binary flip.
        j1_lim = float(g("handover_joint1_limit_rad"))
        j1 = _clamp(az, -j1_lim, j1_lim)
        person_h = float(chosen["person_height_m"])
        # present height = blend(LOW, HIGH). "auto" derives it from the head height;
        # an explicit posture forces it (reliable when the camera can't see the head).
        posture = str(params.get("posture", "auto")).strip().lower()
        if posture == "standing":
            blend = 1.0
        elif posture == "seated":
            blend = 0.0
        else:
            posture = "auto"
            blend = _height_blend(person_h, float(g("handover_person_height_low_m")),
                                  float(g("handover_person_height_high_m")))
        present_pose = _lerp_pose(list(g("handover_present_pose")),
                                  list(g("handover_present_pose_standing")), blend)
        staging = _with_joint1(list(g("handover_staging_pose")), j1)
        present = _with_joint1(present_pose, j1)
        result_data["joint1_rad"] = round(j1, 4)
        result_data["person_height_m"] = round(person_h, 3)
        result_data["posture"] = posture
        result_data["present_blend"] = round(blend, 3)  # 0=low/seated .. 1=high/standing
        ctx.log(f"[HANDOVER] head {person_h:+.2f}m above camera, posture={posture} "
                f"-> present blend {blend:.2f} (0=low/seated .. 1=high/standing)")

        feedback("ORIENT", 0.55)
        if not ctx.move_arm_to(staging, time_sec=float(g("handover_stage_time_sec"))):
            return fail("arm move to staging pose failed")
        if is_cancelled():
            ctx.move_arm_to(capture_pose, time_sec=float(g("handover_retract_time_sec")))
            return fail("canceled while orienting (object kept)")

        feedback("PRESENT", 0.75)
        if not ctx.move_arm_to(present, time_sec=float(g("handover_present_time_sec"))):
            # retract toward safety before reporting
            ctx.move_arm_to(staging, time_sec=2.0)
            ctx.move_arm_to(capture_pose, time_sec=float(g("handover_retract_time_sec")))
            return fail("arm move to present pose failed")

        # ---- 6. release: dwell, open, retract -----------------------------
        feedback("TAKE_OBJECT", 0.85)
        if not self._dwell(ctx, dwell, is_cancelled):
            # canceled during dwell -> keep the object, retract without opening
            ctx.move_arm_to(staging, time_sec=2.0)
            ctx.move_arm_to(capture_pose, time_sec=float(g("handover_retract_time_sec")))
            return fail("canceled during dwell (object kept)")

        feedback("RELEASE", 0.9)
        open_pos = float(ctx.get_param("gripper_open_position", 0.07))
        max_effort = float(ctx.get_param("gripper_max_effort", 5.0))
        opened = ctx.set_gripper(open_pos, max_effort=max_effort)
        result_data["released"] = bool(opened)
        ctx.sleep(0.5)

        feedback("RETRACT", 0.95)
        ctx.move_arm_to(staging, time_sec=2.0)
        ctx.move_arm_to(capture_pose, time_sec=float(g("handover_retract_time_sec")))
        feedback("DONE", 1.0)

        result_data["gripper_width"] = round(float(ctx.gripper_width), 4)
        if not opened:
            return SkillResult(False, "presented the object but the gripper "
                                      "open command failed", result_data)
        return SkillResult(
            True,
            f"handed object to the person at {dist:.2f}m "
            f"({math.degrees(az):.0f}deg); returned to ready",
            result_data)

    # --- best-effort cleanup -------------------------------------------------
    def _quiet_gate_off(self, ctx: SkillContext) -> None:
        """Stop the visual servo from driving the arm (best effort, short wait)."""
        for name in ("grasp_enabled", "grasp_auto_loop", "place_mode",
                     "use_external_target"):
            try:
                ctx.set_bool_param(name, False, timeout=0.5)
            except Exception:
                pass
        try:
            ctx.publish_prompt("")
        except Exception:
            pass

    def _dwell(self, ctx: SkillContext, seconds: float,
               is_cancelled: Callable[[], bool]) -> bool:
        """Sleep in small slices so a cancel during the dwell is responsive.

        Returns False if canceled (caller keeps the object), True otherwise.
        """
        end = ctx.now() + max(0.0, seconds)
        while ctx.ok() and ctx.now() < end:
            if is_cancelled():
                return False
            ctx.sleep(0.1)
        return True

    # --- hardware I/O (kept thin + isolated so unit tests monkeypatch them) --
    def _capture_frame(self, serial: str, width: int, height: int, fps: int,
                       warmup: int, timeout_sec: float, extra_site: str = "",
                       retries: int = 2, reset_on_fail: bool = True,
                       reset_wait_sec: float = 7.0):
        """Grab one aligned (color, depth_m, intrinsics) from the D435i, on demand.

        Returns (color_bgr[H,W,3] uint8, depth_m[H,W] float32 meters, intr) or
        None. ``intr`` = (fx, fy, ppx, ppy, w, h). The D435i can drop into a
        no-frames state under rapid open/close cycling, so a failed grab is retried:
        a cheap re-open first, then a hardware reset (RealSense USB quirk).
        """
        import time

        rs = _lazy_import("pyrealsense2", extra_site)  # hardware-only dep; lazy
        last_err = None
        for attempt in range(int(retries) + 1):
            try:
                return self._grab_once(rs, serial, width, height, fps, warmup,
                                       timeout_sec)
            except RuntimeError as e:  # e.g. "Frame didn't arrive within 5000"
                last_err = e
                if attempt >= int(retries):
                    break
                # Escalate: cheap re-open on the first retry, hardware reset after.
                if reset_on_fail and serial and attempt >= 1:
                    self._reset_device(rs, serial)
                    time.sleep(float(reset_wait_sec))  # wait for USB re-enumeration
                else:
                    time.sleep(1.5)
        raise last_err if last_err else RuntimeError("D435i capture failed")

    def _grab_once(self, rs, serial, width, height, fps, warmup, timeout_sec):
        """One open -> warmup -> aligned grab -> stop. Raises on a frame timeout."""
        pipeline = rs.pipeline()
        config = rs.config()
        if serial:
            config.enable_device(str(serial))
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        profile = pipeline.start(config)
        try:
            depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            align = rs.align(rs.stream.color)
            timeout_ms = max(1000, int(timeout_sec * 1000))
            frames = None
            for _ in range(max(1, warmup)):
                frames = pipeline.wait_for_frames(timeout_ms)
            aligned = align.process(frames)
            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()
            if not depth_frame or not color_frame:
                return None
            color = np.asanyarray(color_frame.get_data())            # H,W,3 BGR uint8
            depth_raw = np.asanyarray(depth_frame.get_data())        # H,W uint16
            depth_m = depth_raw.astype("float32") * float(depth_scale)
            ci = color_frame.profile.as_video_stream_profile().intrinsics
            intr = (float(ci.fx), float(ci.fy), float(ci.ppx), float(ci.ppy),
                    int(ci.width), int(ci.height))
            return color, depth_m, intr
        finally:
            pipeline.stop()

    def _reset_device(self, rs, serial):
        """Hardware-reset the D435i to recover the no-frames USB state (best effort)."""
        try:
            for d in rs.context().query_devices():
                if d.get_info(rs.camera_info.serial_number) == str(serial):
                    d.hardware_reset()
                    return
        except Exception:  # reset is a recovery attempt; never mask the real error
            pass

    def _detect_person(self, color_bgr, url: str, prompt: str,
                       timeout: float, extra_site: str = "") \
            -> Tuple[List[Dict[str, Any]], int, int]:
        """POST the frame to the Grounding DINO HTTP service; return detections.

        Returns (detections, image_width, image_height). Each detection has
        cx, cy, w, h (pixels), score, class_id. Raises on transport/HTTP error.
        """
        cv2 = _lazy_import("cv2", extra_site)  # heavy dep; only needed on the robot

        ok, buf = cv2.imencode(".jpg", color_bgr)
        if not ok:
            raise RuntimeError("failed to JPEG-encode the D435i frame")
        img_b64 = base64.b64encode(buf.tobytes()).decode("ascii")
        payload = json.dumps({"prompt": prompt, "image": img_b64}).encode("utf-8")
        req = urlrequest.Request(
            url, data=payload, headers={"Content-Type": "application/json"},
            method="POST")
        try:
            with urlrequest.urlopen(req, timeout=timeout) as resp:
                data = json.loads(resp.read().decode("utf-8"))
        except urlerror.URLError as e:
            raise RuntimeError(f"detector unreachable at {url}: {e}")
        if not isinstance(data, dict):
            raise RuntimeError("detector returned a non-object response")
        return (list(data.get("detections", [])),
                data.get("image_width"), data.get("image_height"))


# --- helpers ---------------------------------------------------------------

def _lazy_import(name: str, extra_site: str = ""):
    """Import a runtime-only dep, falling back to an extra site-packages path.

    On Jetson the ROS env runs with PYTHONNOUSERSITE=1 (numpy pinned to the ROS
    build), so the user-site pyrealsense2 / cv2 are not importable. numpy is
    already loaded by the time this runs, so appending the user site at runtime
    exposes those modules WITHOUT swapping the loaded numpy. A no-op normal import
    when ``extra_site`` is empty or the module is already on the path.
    """
    try:
        return __import__(name)
    except ImportError:
        if not extra_site:
            raise
        import sys
        if extra_site not in sys.path:
            sys.path.append(extra_site)
        return __import__(name)


# --- pure helpers (no ROS / no hardware -> unit-testable) -------------------

def _select_people(dets: List[Dict[str, Any]], img_w: int, img_h: int,
                   score_min: float, min_box_height_frac: float) -> List[Dict[str, Any]]:
    """Filter person detections and sort largest (nearest) first.

    Keeps boxes whose label is/contains "person" (lenient: empty label accepted),
    score >= score_min, and height >= min_box_height_frac of the image — small or
    spurious boxes are dropped so we never present to a distant/false positive.
    """
    valid: List[Dict[str, Any]] = []
    for d in dets:
        try:
            cx = float(d["cx"])
            cy = float(d["cy"])
            w = float(d["w"])
            h = float(d["h"])
            score = float(d.get("score", 0.0))
        except (KeyError, TypeError, ValueError):
            continue
        cls = str(d.get("class_id", "")).strip().lower()
        if cls and "person" not in cls:
            continue
        if score < score_min:
            continue
        if h < min_box_height_frac * img_h:
            continue
        valid.append({"cx": cx, "cy": cy, "w": w, "h": h,
                      "score": score, "area": w * h, "class_id": cls})
    valid.sort(key=lambda p: p["area"], reverse=True)
    return valid


def _ambiguous(people: List[Dict[str, Any]], img_w: int, ratio: float) -> bool:
    """True if the two largest people are similarly sized AND far apart laterally.

    A second person nearly as large as the first, on the other side of the frame,
    means we can't tell who the recipient is — better to abort than present to the
    wrong one. Two people side by side (similar cx) is NOT ambiguous (same way).
    """
    if len(people) < 2:
        return False
    a, b = people[0], people[1]
    similar = b["area"] > ratio * a["area"]
    far_apart = abs(b["cx"] - a["cx"]) > 0.20 * max(1, img_w)
    return bool(similar and far_apart)


def _torso_pixel(person: Dict[str, Any], torso_frac: float) -> Tuple[float, float]:
    """Upper-torso sample pixel: down ``torso_frac`` of the box from its top edge.

    Sampling the torso (not the bbox center, which on a standing person is near the
    waist, and not the edges, which mix in background) gives a stable body depth.
    """
    y_top = person["cy"] - person["h"] * 0.5
    return person["cx"], y_top + torso_frac * person["h"]


def _depth_median_roi(depth_m, px: float, py: float, half: int,
                      min_pixels: int, min_depth_m: float) -> Optional[float]:
    """Median of valid depths in a square ROI around (px, py); None if too sparse.

    Drops zeros/NaNs and anything closer than min_depth_m (sensor noise / holes).
    Requires >= min_pixels valid samples so a couple of stray pixels can't set a
    bogus distance.
    """
    h_img, w_img = depth_m.shape[:2]
    cx_i, cy_i = int(round(px)), int(round(py))
    x0 = max(0, cx_i - half)
    x1 = min(w_img, cx_i + half + 1)
    y0 = max(0, cy_i - half)
    y1 = min(h_img, cy_i + half + 1)
    if x1 <= x0 or y1 <= y0:
        return None
    roi = depth_m[y0:y1, x0:x1]
    vals = roi[(roi > float(min_depth_m)) & np.isfinite(roi)]
    if vals.size < int(min_pixels):
        return None
    return float(np.median(vals))


def _deproject(px: float, py: float, z: float,
               intr: Tuple[float, float, float, float, int, int]) -> Tuple[float, float, float]:
    """Pinhole back-projection of a pixel + depth to the camera optical frame.

    intr = (fx, fy, ppx, ppy, w, h). Returns (X right, Y down, Z forward) in
    metres. Distortion is ignored — negligible at handover range, and the mount
    extrinsic dominates the error budget anyway.
    """
    fx, fy, ppx, ppy = intr[0], intr[1], intr[2], intr[3]
    x = (px - ppx) / fx * z
    y = (py - ppy) / fy * z
    return x, y, z


def _horizontal_forward(z_opt: float, y_opt: float, roll: float) -> float:
    """Forward distance in the gravity-horizontal plane, correcting camera tilt.

    The D435i usually looks slightly up or down (tilt ``roll`` about its right/x
    optical axis: roll > 0 = up, roll < 0 = down). Leveling that tilt mixes the
    optical forward (z) and vertical (y) — without it, a downward-tilted camera
    reports the slant range and over-estimates how far ahead the person is. The
    lateral (x) is on the rotation axis, so it (and therefore ``left``) is
    unaffected and is handled separately by _optical_to_base.
    """
    return z_opt * math.cos(roll) + y_opt * math.sin(roll)


def _height_above_camera(z_opt: float, y_opt: float, roll: float) -> float:
    """Torso height above the camera optical centre, in metres (gravity-leveled).

    Same tilt-leveling as _horizontal_forward but the vertical component: optical y
    is down, so after removing the camera's up/down tilt (roll about x) the height
    above the camera is z·sin(roll) − y·cos(roll). Positive = above the camera (a
    standing person's torso); near zero = at camera height; negative = below. Used
    to pick the standing vs seated present pose. (Threshold is relative to the
    camera, so it needs no separate camera-height calibration.)
    """
    return z_opt * math.sin(roll) - y_opt * math.cos(roll)


def _optical_to_base(forward: float, left: float, cam_x: float, cam_y: float,
                     cam_yaw: float) -> Tuple[float, float]:
    """Map a camera-horizontal (forward, left) point to the arm-base XY plane.

    The D435i has no TF, so we use a horizontal extrinsic: the camera sits at
    (cam_x, cam_y) in the base frame with its forward axis yawed cam_yaw from base
    +X (CCW positive about base +Z). Returns the point's (x, y) in the base frame.
    """
    c, s = math.cos(cam_yaw), math.sin(cam_yaw)
    # camera forward unit = (c, s); camera left unit = (-s, c)
    px = cam_x + forward * c - left * s
    py = cam_y + forward * s + left * c
    return px, py


def _with_joint1(pose: List[float], joint1: float) -> List[float]:
    """Copy a 6-joint pose with joint1 (base yaw) overwritten by the person azimuth."""
    out = [float(v) for v in pose]
    if not out:
        return [joint1]
    out[0] = float(joint1)
    return out


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _height_blend(torso_h: float, low_h: float, high_h: float) -> float:
    """Blend factor 0..1 for the present height from the torso height.

    torso_h <= low_h -> 0 (use the LOW pose); >= high_h -> 1 (HIGH pose); linear in
    between. Degenerate band (high <= low) -> 0 (always LOW), so a mis-set band
    can never extend higher than taught.
    """
    span = high_h - low_h
    if span <= 1e-6:
        return 0.0
    return _clamp((torso_h - low_h) / span, 0.0, 1.0)


def _lerp_pose(low: List[float], high: List[float], t: float) -> List[float]:
    """Element-wise joint interpolation: low + t*(high-low). t is clamped 0..1.

    Interpolating in joint space between two reachable taught poses stays in the
    arm's workspace (no IK), and the endpoints are exactly the taught poses.
    """
    t = _clamp(t, 0.0, 1.0)
    n = min(len(low), len(high))
    return [float(low[i]) + (float(high[i]) - float(low[i])) * t for i in range(n)]
