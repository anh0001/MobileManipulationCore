#!/usr/bin/env python3
# Copyright 2026 MobileManipulationCore Contributors
# Licensed under the Apache License, Version 2.0.
"""Dry-run the handover skill's PERCEPTION half — no ROS, no arm motion.

Grabs one D435i frame, detects a person via the Grounding DINO HTTP service, and
prints the computed azimuth / distance / joint1 (and whether the safety gates
would pass), using the calibrated extrinsic from config/handover_params.yaml. It
saves an annotated image so you can eyeball the detection + torso depth point.

Use it to CALIBRATE the D435i mount: stand a person straight ahead at a known
distance and tune handover_camera_yaw_rad until azimuth ~ 0, and
handover_camera_roll_rad until distance matches the tape measure.

    python3 scripts/handover_dryrun.py
    python3 scripts/handover_dryrun.py --out /tmp/h.jpg --loop
"""
import argparse
import math
import os
import sys

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "src", "manipulation_policy"))

from manipulation_policy.skills.handover_skill import (  # noqa: E402
    HANDOVER_PARAM_DEFAULTS, HandoverSkill, _ambiguous, _clamp, _deproject,
    _depth_median_roi, _horizontal_forward, _optical_to_base, _select_people,
    _torso_pixel)


def load_params():
    """handover_* params from config/handover_params.yaml, over the code defaults."""
    params = dict(HANDOVER_PARAM_DEFAULTS)
    cfg_path = os.path.join(REPO, "config", "handover_params.yaml")
    try:
        with open(cfg_path, "r", encoding="utf-8") as fh:
            doc = yaml.safe_load(fh) or {}
        ros_params = (doc.get("skill_server", {}) or {}).get("ros__parameters", {}) or {}
        for k, v in ros_params.items():
            if k in params:
                params[k] = v
        print(f"[cfg] loaded overrides from {cfg_path}")
    except FileNotFoundError:
        print(f"[cfg] {cfg_path} not found; using code defaults")
    return params


def run_once(skill, p, out_path):
    import cv2

    cap = skill._capture_frame(
        serial=str(p["handover_camera_serial"]),
        width=int(p["handover_color_width"]),
        height=int(p["handover_color_height"]),
        fps=int(p["handover_fps"]),
        warmup=int(p["handover_warmup_frames"]),
        timeout_sec=float(p["handover_capture_timeout_sec"]))
    if cap is None:
        print("[capture] FAILED: no aligned frame")
        return
    color, depth_m, intr = cap
    print(f"[capture] color={color.shape} depth={depth_m.shape} "
          f"intr fx={intr[0]:.1f} fy={intr[1]:.1f} ppx={intr[2]:.1f} ppy={intr[3]:.1f}")

    dets, img_w, img_h = skill._detect_person(
        color, url=str(p["handover_detect_url"]),
        prompt=str(p["handover_detect_prompt"]),
        timeout=float(p["handover_detect_timeout_sec"]))
    img_w = int(img_w or color.shape[1])
    img_h = int(img_h or color.shape[0])
    people = _select_people(dets, img_w, img_h,
                            score_min=float(p["handover_score_min"]),
                            min_box_height_frac=float(p["handover_min_box_height_frac"]))
    print(f"[detect] raw={len(dets)} valid_people={len(people)} "
          f"ambiguous={_ambiguous(people, img_w, float(p['handover_ambiguous_area_ratio']))}")
    if not people:
        cv2.imwrite(out_path, color)
        print(f"[detect] no person; saved raw frame -> {out_path}")
        return

    cam_x = float(p["handover_camera_x_m"])
    cam_y = float(p["handover_camera_y_m"])
    cam_yaw = float(p["handover_camera_yaw_rad"])
    cam_roll = float(p["handover_camera_roll_rad"])
    torso_frac = float(p["handover_torso_frac"])
    roi_half = int(p["handover_depth_roi_half_px"])
    min_px = int(p["handover_min_depth_pixels"])
    min_depth = float(p["handover_min_valid_depth_m"])

    chosen = None
    for person in people:
        tx, ty = _torso_pixel(person, torso_frac)
        z = _depth_median_roi(depth_m, tx, ty, roi_half, min_px, min_depth)
        if z is None:
            continue
        x_opt, y_opt, z_opt = _deproject(tx, ty, z, intr)
        fwd = _horizontal_forward(z_opt, y_opt, cam_roll)
        px, py = _optical_to_base(forward=fwd, left=-x_opt,
                                  cam_x=cam_x, cam_y=cam_y, cam_yaw=cam_yaw)
        chosen = dict(person=person, tx=tx, ty=ty, depth=z,
                      base_xy=(px, py), az=math.atan2(py, px), dist=math.hypot(px, py))
        break

    # annotate every valid person + the chosen torso point
    for person in people:
        x1 = int(person["cx"] - person["w"] / 2)
        y1 = int(person["cy"] - person["h"] / 2)
        x2 = int(person["cx"] + person["w"] / 2)
        y2 = int(person["cy"] + person["h"] / 2)
        cv2.rectangle(color, (x1, y1), (x2, y2), (0, 180, 0), 2)

    if chosen is None:
        cv2.imwrite(out_path, color)
        print("[localize] person(s) found but NO valid torso depth (out of range / occluded)")
        print(f"[saved] {out_path}")
        return

    az, dist = chosen["az"], chosen["dist"]
    j1 = _clamp(az, -float(p["handover_joint1_limit_rad"]), float(p["handover_joint1_limit_rad"]))
    # gate evaluation (would the real skill proceed?)
    reasons = []
    if dist < float(p["handover_min_distance_m"]):
        reasons.append(f"too close (<{p['handover_min_distance_m']}m)")
    if dist > float(p["handover_max_distance_m"]):
        reasons.append(f"too far (>{p['handover_max_distance_m']}m)")
    max_az_deg = math.degrees(float(p["handover_max_azimuth_rad"]))
    if abs(az) > float(p["handover_max_azimuth_rad"]):
        reasons.append(f"too far to the side (>{max_az_deg:.0f}deg)")
    verdict = "WOULD PRESENT" if not reasons else "WOULD ABORT: " + ", ".join(reasons)

    print(f"[localize] torso_px=({chosen['tx']:.0f},{chosen['ty']:.0f}) "
          f"depth={chosen['depth']:.3f}m")
    print(f"[result ] base_xy=({chosen['base_xy'][0]:.3f},{chosen['base_xy'][1]:.3f}) "
          f"distance={dist:.3f}m azimuth={math.degrees(az):.1f}deg joint1={j1:.3f}rad "
          f"score={chosen['person']['score']:.2f}")
    print(f"[gate   ] {verdict}")

    tx, ty = int(chosen["tx"]), int(chosen["ty"])
    cv2.circle(color, (tx, ty), 6, (0, 0, 255), -1)
    cv2.putText(color, f"d={dist:.2f}m az={math.degrees(az):.0f}deg", (tx + 8, ty),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.putText(color, verdict, (10, img_h - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (0, 200, 0) if not reasons else (0, 0, 255), 2)
    cv2.imwrite(out_path, color)
    print(f"[saved  ] {out_path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/handover_dryrun.jpg")
    ap.add_argument("--loop", action="store_true", help="repeat until Ctrl-C")
    args = ap.parse_args()

    p = load_params()
    print(f"[cfg] camera x={p['handover_camera_x_m']} y={p['handover_camera_y_m']} "
          f"yaw={p['handover_camera_yaw_rad']} roll={p['handover_camera_roll_rad']}")
    skill = HandoverSkill()
    if args.loop:
        import time
        while True:
            run_once(skill, p, args.out)
            print("-" * 60)
            time.sleep(1.0)
    else:
        run_once(skill, p, args.out)


if __name__ == "__main__":
    main()
