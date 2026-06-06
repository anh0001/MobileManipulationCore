# Copyright 2026 MobileManipulationCore Contributors
# Licensed under the Apache License, Version 2.0.
"""localize_object skill — report the 3D position of named objects.

Answers "where is the banana?" — NOT "where is the robot?". This is OBJECT
localization (open-vocabulary detect -> depth -> 3D point in a robot frame), and
is deliberately named ``localize_object`` to keep it distinct from robot
self/navigation localization (AMCL / FASTLIO "where am I on the map"), which is a
different subsystem entirely.

One call returns each requested object's (x, y, z) in ``base_footprint`` (or any
TF frame). It works through either depth camera:

  * ``wrist`` — the RealSense **D405** on the arm. Color+depth+intrinsics come from
    its ROS topics and the camera-optical -> output-frame transform is a LIVE TF
    lookup, so the answer tracks the arm pose. Most accurate. The object must be
    in the wrist camera's view, so raise the arm to a look-down pose first
    (the ``arm_ready_pose`` skill) if it is parked.
  * ``rear`` — the fixed RealSense **D435i** behind the arm. It has no ROS node and
    no TF frame, so (exactly like the handover skill) it is grabbed on demand by
    serial with pyrealsense2 and lifted into ``base_footprint`` with a MANUAL,
    calibrated extrinsic. Until that extrinsic is measured the rear numbers are
    approximate — the result carries a ``calibration`` flag saying so.

Pipeline (shared by both cameras)
---------------------------------
  1. Acquire one aligned color + depth frame (+ intrinsics) from the camera.
  2. For each requested label: POST the color frame to the existing Grounding DINO
     HTTP detector (the same service pick/handover use — no new model) and take
     the most confident box. If nothing is found and the optional Claude vision
     fallback is enabled, ask it for the object's pixel.
  3. Sample a robust median depth in a small ROI at the box centre.
  4. Deproject (pixel + depth) to a 3D point in the camera optical frame.
  5. Lift that point into the output frame — live TF for the wrist, the manual
     extrinsic for the rear.

The front fisheye has no depth, so it cannot give a 3D position and is rejected
with a clear message.

Runtime deps: the Grounding DINO HTTP detector must be up (brought up with the
visual-servo bringup). For the rear camera, pyrealsense2 + cv2 must be importable
(hardware-only). The optional Claude fallback needs the ``anthropic`` package and
ANTHROPIC_API_KEY. All degrade gracefully with a clear message when absent.

CALIBRATION before trusting rear-camera numbers (config/localize_object_params.yaml)
-----------------------------------------------------------------------------------
  * ``localize_rear_camera_{x,y,z}_m`` and ``localize_rear_camera_{yaw,pitch}_rad``
    place the D435i optical frame in ``base_footprint``. Measure them, then set
    ``localize_rear_calibrated: true`` so the result stops flagging the pose as
    uncalibrated.
"""
from __future__ import annotations

import base64
import json
import math
import os
from typing import Any, Callable, Dict, List, Optional, Tuple
from urllib import error as urlerror
from urllib import request as urlrequest

import numpy as np

from .base import Skill, SkillParam, SkillResult, SkillContext
from .registry import register_skill

# Server tunables (declared by skill_server via Skill.server_params; overridable
# from config/localize_object_params.yaml and at runtime via `ros2 param set`).
# Floats are written x.0 so the ROS parameter type is DOUBLE — keep config
# overrides type-consistent.
LOCALIZE_PARAM_DEFAULTS: Dict[str, Any] = {
    # --- output frame ---
    "localize_output_frame": "base_footprint",
    # --- detection (reuses the Grounding DINO HTTP service) ---
    "localize_detect_url": "http://localhost:30543/detect",
    "localize_detect_timeout_sec": 8.0,
    "localize_score_min": 0.30,
    "localize_max_detections": 5,
    # --- robust depth sampling at the box centre ---
    "localize_depth_roi_half_px": 8,
    "localize_min_depth_pixels": 20,
    "localize_min_valid_depth_m": 0.12,
    "localize_max_valid_depth_m": 3.0,
    # --- wrist D405: ROS topics + live TF (most accurate) ---
    # NOTE: color is image_raw (not image_rect_raw): that is what the realsense
    # node actually publishes on this robot and what the visual_servo_node uses
    # for rgb_topic. The D405's color is near-rectified and its depth is registered
    # to it, so color image_raw + depth image_rect_raw is the same proven pair the
    # grasp pipeline samples. (image_rect_raw is the C++ default but is NOT
    # published here, so subscribing to it yields zero frames.)
    "localize_wrist_rgb_topic": "/piper/wrist_camera/piper_d405/color/image_raw",
    "localize_wrist_depth_topic": "/piper/wrist_camera/piper_d405/depth/image_rect_raw",
    "localize_wrist_info_topic": "/piper/wrist_camera/piper_d405/color/camera_info",
    "localize_wrist_optical_frame": "piper_camera_optical_frame",
    "localize_wrist_frame_wait_sec": 5.0,
    # --- rear D435i: pyrealsense2 on demand (mirrors the handover capture) ---
    # Extra site-packages so pyrealsense2/cv2/anthropic import under the ROS env
    # (PYTHONNOUSERSITE=1 on Jetson). "" = import normally. Appended AFTER numpy
    # is already loaded so the pinned ROS numpy is unchanged.
    "localize_extra_site_packages": "",
    "localize_rear_camera_serial": "243722070013",
    "localize_rear_color_width": 640,
    "localize_rear_color_height": 480,
    "localize_rear_fps": 30,
    "localize_rear_warmup_frames": 12,
    "localize_rear_capture_timeout_sec": 5.0,
    "localize_rear_capture_retries": 2,
    "localize_rear_capture_reset_on_fail": True,
    "localize_rear_capture_reset_wait_sec": 7.0,
    # --- rear D435i extrinsic: optical frame pose in base_footprint — CALIBRATE ---
    # Optical axes x=right, y=down, z=forward. (x, y, z) is the camera position in
    # base_footprint; yaw rotates its forward axis about base +Z (CCW+); pitch tilts
    # it down (+) / up (-). Defaults are a behind-the-arm guess — MEASURE them.
    "localize_rear_camera_x_m": -0.12,
    "localize_rear_camera_y_m": -0.18,
    "localize_rear_camera_z_m": 0.30,
    "localize_rear_camera_yaw_rad": 0.0,
    "localize_rear_camera_pitch_rad": 0.0,
    "localize_rear_calibrated": False,
    # --- optional Claude vision fallback when the detector finds nothing ---
    "localize_vlm_fallback": False,
    "localize_vlm_model": "claude-sonnet-4-6",
    "localize_vlm_timeout_sec": 20.0,
}


@register_skill
class LocalizeObjectSkill(Skill):
    name = "localize_object"
    description = (
        "Report the 3D position of one or more named objects relative to the robot "
        "base (base_footprint). This locates OBJECTS in the scene — it is not robot "
        "self-localization. Detects each object by open-vocabulary name through a "
        "depth camera ('wrist' D405 on the arm, or 'rear' D435i behind the arm), "
        "samples depth, and returns (x, y, z) in metres per object. Use for "
        "questions like 'where is the banana?' or 'what is the 3D position of the "
        "red cup and the bottle from the rear camera?'. For the wrist camera the "
        "object must be in view (raise the arm to the ready look-down pose first)."
    )
    params = [
        SkillParam("objects", "string", required=True,
                   description='one or more object labels to locate, comma-separated '
                               '(e.g. "banana" or "banana, red cup, bottle"). '
                               "Open-vocabulary."),
        SkillParam("camera", "string", default="wrist",
                   description='depth camera to look through: "wrist" (D405 on the '
                               'arm — live TF, most accurate; object must be in view) '
                               'or "rear" (fixed D435i behind the arm — calibrated '
                               "extrinsic)."),
        SkillParam("frame", "string", default="",
                   description="output TF frame for the returned positions; empty "
                               "uses the server default (base_footprint)."),
        SkillParam("max_per_object", "integer", default=1,
                   description="max instances to return per label (1 = the single "
                               "most confident detection)."),
        SkillParam("timeout_sec", "number", default=0.0,
                   description="reserved; capture and detection have their own "
                               "timeouts."),
    ]
    server_params = LOCALIZE_PARAM_DEFAULTS

    def execute(self, ctx: SkillContext, params: Dict[str, Any],
                feedback: Callable[[str, float], None],
                is_cancelled: Callable[[], bool]) -> SkillResult:
        g = lambda n: ctx.get_param(n, LOCALIZE_PARAM_DEFAULTS[n])  # noqa: E731

        objects = _parse_objects(params.get("objects", ""))
        camera = _norm_camera(str(params.get("camera") or "wrist"))
        out_frame = (str(params.get("frame") or "").strip()
                     or str(g("localize_output_frame")))
        max_per = max(1, int(params.get("max_per_object") or 1))
        extra_site = str(g("localize_extra_site_packages"))

        result_data: Dict[str, Any] = {
            "frame": out_frame, "camera": camera, "objects": []}

        if not objects:
            return SkillResult(False, "no object labels given", result_data)
        if camera == "front":
            return SkillResult(
                False, "the front fisheye has no depth; use the 'wrist' or 'rear' "
                       "camera for a 3D object position", result_data)
        if camera not in ("wrist", "rear"):
            return SkillResult(
                False, f"unknown camera '{camera}'; use 'wrist' or 'rear'",
                result_data)

        # ---- 1. acquire a color+depth frame + a lifter into the output frame ----
        feedback("CAPTURE", 0.1)
        try:
            if camera == "wrist":
                acq = self._acquire_wrist(ctx, g)
            else:
                acq = self._acquire_rear(ctx, g, extra_site)
        except Exception as e:  # hardware / decode error
            return SkillResult(False, f"{camera} camera capture failed: {e}",
                               result_data)
        if acq.get("error"):
            return SkillResult(False, acq["error"], result_data)
        color_bgr, depth_m, intr = acq["color"], acq["depth"], acq["intr"]
        result_data["calibration"] = acq["calibration"]
        if is_cancelled():
            return SkillResult(False, "canceled after capture", result_data)

        # Lift a camera-optical point (x,y,z) into the output frame, or None.
        if camera == "wrist":
            optical_frame = acq["optical_frame"]

            def lift(p):  # live TF, captures the arm pose at lookup time
                return ctx.transform_point(out_frame, optical_frame, p)
        else:
            ext = acq["extrinsic"]

            def lift(p):
                bp = _optical_to_base_3d(p[0], p[1], p[2],
                                         (ext["x"], ext["y"], ext["z"]),
                                         ext["yaw"], ext["pitch"])
                if out_frame == "base_footprint":
                    return bp
                return ctx.transform_point(out_frame, "base_footprint", bp)

        # ---- 2. JPEG-encode the frame once for the detector --------------------
        try:
            cv2 = _lazy_import("cv2", extra_site)
            ok, jpg = cv2.imencode(".jpg", color_bgr)
            if not ok:
                return SkillResult(False, "failed to JPEG-encode the camera frame",
                                   result_data)
            img_b64 = base64.b64encode(jpg.tobytes()).decode("ascii")
        except Exception as e:
            return SkillResult(False, f"image encode unavailable ({e}); cv2 is "
                                      "required for object localization", result_data)

        # ---- 3. locate each requested object -----------------------------------
        found_any = False
        n = len(objects)
        for i, label in enumerate(objects):
            if is_cancelled():
                return SkillResult(False, "canceled during detection", result_data)
            feedback("DETECT", 0.3 + 0.6 * (i / max(1, n)))
            entry = self._locate_one(label, img_b64, color_bgr, depth_m, intr,
                                     lift, g, extra_site, max_per)
            result_data["objects"].append(entry)
            found_any = found_any or bool(entry.get("found"))

        feedback("DONE", 1.0)
        located = [o for o in result_data["objects"] if o.get("found")]
        if found_any:
            parts = [
                f"{o['label']} ({o['position']['x']:.2f}, {o['position']['y']:.2f}, "
                f"{o['position']['z']:.2f})" for o in located]
            note = ("" if result_data["calibration"].startswith("tf")
                    else " [rear extrinsic" +
                         ("" if "UNCALIBRATED" not in result_data["calibration"]
                          else " UNCALIBRATED — approximate") + "]")
            return SkillResult(
                True,
                f"located {len(located)}/{n} object(s) in {out_frame} via the "
                f"{camera} camera: " + "; ".join(parts) + note,
                result_data)
        return SkillResult(
            False,
            f"none of {n} object(s) located via the {camera} camera "
            "(not detected, or no valid depth at the object)",
            result_data)

    # ---- acquisition -------------------------------------------------------
    def _acquire_wrist(self, ctx: SkillContext, g) -> Dict[str, Any]:
        """Latest wrist D405 color+depth+intrinsics over ROS, lifted via live TF."""
        wait = float(g("localize_wrist_frame_wait_sec"))
        optical_frame = str(g("localize_wrist_optical_frame"))
        end = ctx.now() + max(0.5, wait)
        frames = None
        while ctx.ok() and ctx.now() < end:
            frames = ctx.latest_camera_frame("wrist")
            if frames is not None:
                break
            ctx.sleep(0.05)
        if frames is None:
            return {"error": "wrist camera frames not available over ROS — is the "
                             "D405 realsense node up (visual-servo bringup)?"}
        color_msg, depth_msg, info_msg = frames
        color_bgr = _image_msg_to_bgr(color_msg)
        depth_m = _depth_msg_to_meters(depth_msg)
        intr = _caminfo_intrinsics(info_msg)
        return {"color": color_bgr, "depth": depth_m, "intr": intr,
                "optical_frame": optical_frame, "calibration": "tf"}

    def _acquire_rear(self, ctx: SkillContext, g, extra_site: str) -> Dict[str, Any]:
        """One on-demand D435i color+depth grab (pyrealsense2) + manual extrinsic."""
        captured = self._capture_rear_frame(
            serial=str(g("localize_rear_camera_serial")),
            width=int(g("localize_rear_color_width")),
            height=int(g("localize_rear_color_height")),
            fps=int(g("localize_rear_fps")),
            warmup=int(g("localize_rear_warmup_frames")),
            timeout_sec=float(g("localize_rear_capture_timeout_sec")),
            extra_site=extra_site,
            retries=int(g("localize_rear_capture_retries")),
            reset_on_fail=bool(g("localize_rear_capture_reset_on_fail")),
            reset_wait_sec=float(g("localize_rear_capture_reset_wait_sec")))
        if captured is None:
            return {"error": "rear D435i returned no aligned color+depth frame"}
        color_bgr, depth_m, intr = captured
        calibrated = bool(g("localize_rear_calibrated"))
        extrinsic = {
            "x": float(g("localize_rear_camera_x_m")),
            "y": float(g("localize_rear_camera_y_m")),
            "z": float(g("localize_rear_camera_z_m")),
            "yaw": float(g("localize_rear_camera_yaw_rad")),
            "pitch": float(g("localize_rear_camera_pitch_rad")),
        }
        return {"color": color_bgr, "depth": depth_m, "intr": intr,
                "extrinsic": extrinsic,
                "calibration": ("extrinsic" if calibrated
                                else "extrinsic:UNCALIBRATED")}

    # ---- per-object localization ------------------------------------------
    def _locate_one(self, label: str, img_b64: str, color_bgr, depth_m, intr,
                    lift, g, extra_site: str, max_per: int) -> Dict[str, Any]:
        url = str(g("localize_detect_url"))
        timeout = float(g("localize_detect_timeout_sec"))
        score_min = float(g("localize_score_min"))
        max_det = int(g("localize_max_detections"))
        roi_half = int(g("localize_depth_roi_half_px"))
        min_px = int(g("localize_min_depth_pixels"))
        min_d = float(g("localize_min_valid_depth_m"))
        max_d = float(g("localize_max_valid_depth_m"))

        try:
            dets = _detect(url, img_b64, label, timeout, max_det)
        except Exception as e:
            return {"label": label, "found": False, "reason": f"detection failed: {e}"}
        dets = [d for d in dets if float(d.get("score", 0.0)) >= score_min]
        dets.sort(key=lambda d: float(d.get("score", 0.0)), reverse=True)

        used_vlm = False
        if not dets and bool(g("localize_vlm_fallback")):
            px = _vlm_locate(color_bgr, label, str(g("localize_vlm_model")),
                             extra_site, float(g("localize_vlm_timeout_sec")))
            if px is not None:
                dets = [{"cx": px[0], "cy": px[1], "w": 24.0, "h": 24.0,
                         "score": 0.0, "class_id": label, "_vlm": True}]
                used_vlm = True
        if not dets:
            return {"label": label, "found": False, "reason": "not detected"}

        instances: List[Dict[str, Any]] = []
        for d in dets[:max_per]:
            cx, cy = float(d["cx"]), float(d["cy"])
            dcx, dcy = _scale_pixel(cx, cy, color_bgr.shape, depth_m.shape)
            z = _depth_median_roi(depth_m, dcx, dcy, roi_half, min_px, min_d, max_d)
            if z is None:
                continue
            x_opt, y_opt, z_opt = _deproject(dcx, dcy, z, intr)
            p = lift((x_opt, y_opt, z_opt))
            if p is None:
                return {"label": label, "found": False,
                        "reason": "could not transform into the output frame "
                                  "(TF unavailable)"}
            inst = {
                "position": {"x": round(p[0], 3), "y": round(p[1], 3),
                             "z": round(p[2], 3)},
                "distance_m": round(math.sqrt(p[0] ** 2 + p[1] ** 2 + p[2] ** 2), 3),
                "depth_m": round(float(z), 3),
                "score": round(float(d.get("score", 0.0)), 3),
                "pixel": [round(cx, 1), round(cy, 1)],
            }
            if d.get("_vlm"):
                inst["via"] = "vlm_fallback"
            instances.append(inst)

        if not instances:
            return {"label": label, "found": False,
                    "reason": "detected but no valid depth at the object "
                              "(out of range or occluded)"}
        out: Dict[str, Any] = {"label": label, "found": True,
                               "num_detections": len(dets)}
        out.update(instances[0])
        if max_per > 1:
            out["instances"] = instances
        if used_vlm:
            out["via"] = "vlm_fallback"
        return out

    # ---- rear D435i capture (mirrors handover_skill; isolated for tests) ---
    def _capture_rear_frame(self, serial: str, width: int, height: int, fps: int,
                            warmup: int, timeout_sec: float, extra_site: str = "",
                            retries: int = 2, reset_on_fail: bool = True,
                            reset_wait_sec: float = 7.0):
        import time as _time
        rs = _lazy_import("pyrealsense2", extra_site)
        last_err = None
        for attempt in range(int(retries) + 1):
            try:
                return self._grab_once(rs, serial, width, height, fps, warmup,
                                       timeout_sec)
            except RuntimeError as e:  # e.g. "Frame didn't arrive within 5000"
                last_err = e
                if attempt >= int(retries):
                    break
                if reset_on_fail and serial and attempt >= 1:
                    self._reset_device(rs, serial)
                    _time.sleep(float(reset_wait_sec))  # USB re-enumeration
                else:
                    _time.sleep(1.5)
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
        except Exception:  # recovery attempt; never mask the real error
            pass


# --- helpers ---------------------------------------------------------------

def _lazy_import(name: str, extra_site: str = ""):
    """Import a runtime-only dep, falling back to an extra site-packages path.

    Mirrors handover_skill: on Jetson the ROS env runs PYTHONNOUSERSITE=1, so a
    user-site pyrealsense2 / cv2 / anthropic is not importable by default. numpy is
    already loaded by the time this runs, so appending the user site exposes those
    modules without swapping the loaded numpy.
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


def _parse_objects(raw: str) -> List[str]:
    """Split a comma/semicolon/newline-separated label string; dedupe, keep order."""
    parts = [p.strip() for p in raw.replace(";", ",").replace("\n", ",").split(",")]
    out: List[str] = []
    for p in parts:
        if p and p.lower() not in [o.lower() for o in out]:
            out.append(p)
    return out


_CAMERA_ALIASES = {
    "arm": "wrist", "hand": "wrist", "gripper": "wrist", "d405": "wrist",
    "back": "rear", "behind": "rear", "fixed": "rear", "d435": "rear",
    "d435i": "rear", "base": "front", "forward": "front", "fisheye": "front",
}


def _norm_camera(name: str) -> str:
    n = (name or "").strip().lower()
    return _CAMERA_ALIASES.get(n, n)


def _detect(url: str, img_b64: str, prompt: str, timeout: float,
            max_detections: int) -> List[Dict[str, Any]]:
    """POST one frame to the Grounding DINO HTTP detector; return its detections.

    Reuses the already-deployed service (same one pick/handover use). Each
    detection has cx, cy, w, h (pixels), score, class_id. Raises on transport
    error so the caller can report it per object.
    """
    payload = json.dumps({
        "prompt": prompt, "image": img_b64,
        "max_detections": int(max_detections)}).encode("utf-8")
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
    return list(data.get("detections", []))


def _vlm_locate(color_bgr, label: str, model: str, extra_site: str,
                timeout: float) -> Optional[Tuple[float, float]]:
    """Optional Claude vision fallback: pixel of ``label`` in the frame, or None.

    Gated by the caller (localize_vlm_fallback) and ANTHROPIC_API_KEY. Best-effort:
    a missing package/key or any parse error simply returns None so the object is
    reported "not detected" rather than crashing the skill.
    """
    if not os.getenv("ANTHROPIC_API_KEY"):
        return None
    try:
        anthropic = _lazy_import("anthropic", extra_site)
        cv2 = _lazy_import("cv2", extra_site)
    except Exception:
        return None
    try:
        ok, buf = cv2.imencode(".jpg", color_bgr)
        if not ok:
            return None
        b64 = base64.b64encode(buf.tobytes()).decode("ascii")
        client = anthropic.Anthropic()
        instruction = (
            f"Find the {label} in this image. Respond with ONLY a JSON object "
            '{"found": true|false, "x": <0..1>, "y": <0..1>} giving the normalized '
            f"center pixel of the single most prominent {label}. No prose.")
        resp = client.messages.create(
            model=model, max_tokens=200,
            timeout=timeout,
            messages=[{"role": "user", "content": [
                {"type": "image", "source": {"type": "base64",
                                             "media_type": "image/jpeg", "data": b64}},
                {"type": "text", "text": instruction}]}])
        text = "".join(getattr(b, "text", "") for b in resp.content
                       if getattr(b, "type", None) == "text")
        data = json.loads(_extract_json(text))
        if not data.get("found"):
            return None
        h, w = color_bgr.shape[:2]
        return (float(data["x"]) * w, float(data["y"]) * h)
    except Exception:
        return None


def _extract_json(text: str) -> str:
    """Pull the first {...} object out of an LLM reply (tolerant of code fences)."""
    s = text.strip()
    if "```" in s:
        s = s.split("```")[1]
        if s.startswith("json"):
            s = s[4:]
    a, b = s.find("{"), s.rfind("}")
    return s[a:b + 1] if a >= 0 and b > a else s


# --- image message decoding (no cv_bridge dependency) ----------------------

def _image_msg_to_bgr(msg) -> np.ndarray:
    """Decode a sensor_msgs/Image colour frame to an (H,W,3) BGR uint8 array."""
    enc = (msg.encoding or "").lower()
    h, w = int(msg.height), int(msg.width)
    rows = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(h, int(msg.step))
    if enc in ("rgb8", "bgr8"):
        img = rows[:, :w * 3].reshape(h, w, 3)
        return img[:, :, ::-1].copy() if enc == "rgb8" else img.copy()
    if enc in ("rgba8", "bgra8"):
        img = rows[:, :w * 4].reshape(h, w, 4)[:, :, :3]
        return img[:, :, ::-1].copy() if enc == "rgba8" else img.copy()
    if enc in ("mono8", ""):
        gray = rows[:, :w].reshape(h, w)
        return np.repeat(gray[:, :, None], 3, axis=2).copy()
    # Unknown encoding: best-effort 3-channel view.
    img = rows[:, :w * 3].reshape(h, w, 3)
    return img.copy()


def _depth_msg_to_meters(msg) -> np.ndarray:
    """Decode a sensor_msgs/Image depth frame to an (H,W) float32 array in metres.

    Handles 16UC1/mono16 (millimetres -> /1000) and 32FC1 (already metres),
    honouring msg.is_bigendian. Invalid pixels stay 0.0.
    """
    enc = (msg.encoding or "").lower()
    h, w = int(msg.height), int(msg.width)
    rows = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(h, int(msg.step))
    if enc in ("16uc1", "mono16", ""):
        dt = np.dtype(np.uint16)
        if msg.is_bigendian:
            dt = dt.newbyteorder(">")
        d = rows[:, :w * 2].copy().view(dt).reshape(h, w)
        return d.astype(np.float32) / 1000.0
    if enc in ("32fc1",):
        dt = np.dtype(np.float32)
        if msg.is_bigendian:
            dt = dt.newbyteorder(">")
        return rows[:, :w * 4].copy().view(dt).reshape(h, w).astype(np.float32)
    raise RuntimeError(f"unsupported depth encoding '{msg.encoding}'")


def _caminfo_intrinsics(msg) -> Tuple[float, float, float, float, int, int]:
    """(fx, fy, ppx, ppy, w, h) from a sensor_msgs/CameraInfo K matrix."""
    k = list(msg.k)  # row-major [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    return (float(k[0]), float(k[4]), float(k[2]), float(k[5]),
            int(msg.width), int(msg.height))


def _scale_pixel(cx: float, cy: float, color_shape, depth_shape) -> Tuple[float, float]:
    """Map a colour-image pixel to depth-image coords when resolutions differ.

    Both RealSense paths align depth to colour at the SAME resolution, so this is
    usually the identity; it is a safety net if a driver publishes mismatched
    sizes.
    """
    ch, cw = color_shape[0], color_shape[1]
    dh, dw = depth_shape[0], depth_shape[1]
    if (ch, cw) == (dh, dw):
        return cx, cy
    return cx * (dw / max(1, cw)), cy * (dh / max(1, ch))


def _depth_median_roi(depth_m, px: float, py: float, half: int, min_pixels: int,
                      min_depth_m: float, max_depth_m: float) -> Optional[float]:
    """Median of valid depths in a square ROI around (px, py); None if too sparse.

    Drops zeros/NaNs and anything outside [min_depth_m, max_depth_m]. Requires
    >= min_pixels valid samples so a few stray pixels can't set a bogus distance.
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
    vals = roi[(roi > float(min_depth_m)) & (roi < float(max_depth_m))
               & np.isfinite(roi)]
    if vals.size < int(min_pixels):
        return None
    return float(np.median(vals))


def _deproject(px: float, py: float, z: float,
               intr: Tuple[float, float, float, float, int, int]) -> Tuple[float, float, float]:
    """Pinhole back-projection of a pixel + depth to the camera optical frame.

    intr = (fx, fy, ppx, ppy, w, h). Returns (X right, Y down, Z forward) in
    metres. Distortion is ignored (negligible for these rectified RealSense
    streams).
    """
    fx, fy, ppx, ppy = intr[0], intr[1], intr[2], intr[3]
    x = (px - ppx) / fx * z
    y = (py - ppy) / fy * z
    return x, y, z


def _optical_to_base_3d(x_opt: float, y_opt: float, z_opt: float,
                        cam_xyz: Tuple[float, float, float],
                        yaw: float, pitch: float) -> Tuple[float, float, float]:
    """Lift a camera-optical 3D point into base_footprint via a manual extrinsic.

    For the rear D435i, which has no TF. Optical axes are x=right, y=down,
    z=forward; base_footprint is x=forward, y=left, z=up. The camera sits at
    ``cam_xyz`` in base_footprint, its forward axis yawed ``yaw`` about base +Z
    (CCW+) and pitched ``pitch`` (down +, up -). Pure trig, no numpy needed.
    """
    # optical -> camera body (REP-103: x forward, y left, z up)
    xb, yb, zb = z_opt, -x_opt, -y_opt
    # pitch about body y (down +): forward tilts toward -z
    cp, sp = math.cos(pitch), math.sin(pitch)
    x1 = xb * cp + zb * sp
    y1 = yb
    z1 = -xb * sp + zb * cp
    # yaw about base z (CCW +)
    cy_, sy_ = math.cos(yaw), math.sin(yaw)
    xr = x1 * cy_ - y1 * sy_
    yr = x1 * sy_ + y1 * cy_
    zr = z1
    return (cam_xyz[0] + xr, cam_xyz[1] + yr, cam_xyz[2] + zr)
