# Hand-eye Calibration (wrist D405 → PiPER arm)

Eye-in-hand calibration of the wrist RealSense D405 relative to the arm TCP
(`piper_tcp`). Produces the fixed `TCP → camera` transform that makes the grasp
estimate accurate **across the workspace** — replacing the hand-tuned
`grasp_offset` fudge.

Script: [`scripts/handeye_orbit_calibrate.py`](../scripts/handeye_orbit_calibrate.py)
Result: [`config/handeye_calibration.yaml`](../config/handeye_calibration.yaml)

## Board

Printed **ChArUco**, calib.io generator:
- 10 columns × 7 rows, **25 mm** checker, ~18 mm marker, **DICT_4X4_50**
- Print **100% / actual size**, glue **flat + rigid**, fix it in view.

## Run

```bash
# 1. bringup (ranger-garden-assistant) + the DINO+SAM server up, arm enabled.
# 2. aim the wrist cam at the board (hand-guide or "3. Arm: Move to capture pose").
# 3. run the orbit calibration:
PYTHONNOUSERSITE=1 python3 scripts/handeye_orbit_calibrate.py
```

It measures the board center (depth-Kabsch), generates camera poses **orbiting**
the board, IK-plans each (collision-checked), executes the reachable ones,
solves `cv2.calibrateHandEye`, and writes the YAML with a convergence flag.

## Why two things were essential (the rest fails)

1. **Look-at ORBIT poses, not wrist-spin.** `calibrateHandEye` needs ≥30°
   (better 60°) rotation about **≥2 non-parallel axes**. Spinning the wrist in
   place rotates mostly about the optical axis (roll) and swings the board out of
   frame → ill-conditioned. Orbiting the camera *around* the board (azimuth ±40°,
   elevation 25/40/55°, distance 0.35/0.43/0.50 m, roll ±25°), each pose aimed at
   the board center, keeps the board in view while the viewing angle changes a
   lot. Sources: HALCON `calibrate_hand_eye`, rc-visard hand-eye docs,
   OpenCV #24871.
2. **Depth-Kabsch board pose, not `solvePnP`.** A flat ChArUco viewed at an angle
   has a **two-solution PnP ambiguity** that flips per-view and corrupts the
   rotation. The D405 has depth → sample depth at each ChArUco corner, deproject
   to 3D, rigid-fit (Kabsch, scale = 1, RANSAC-trimmed) against the board's known
   corner geometry → unambiguous pose (RMS ~1–2 mm).
   (`scripts/handeye_calibrate.py::board_pose_depth`.)

## Reading the result

`config/handeye_calibration.yaml`:
- `converged: true` and `spread_mm < ~5` → trustworthy (solvers agree).
- `translation_xyz_m`, `rotation_rpy_rad`, `rotation_quat_xyzw` = `piper_tcp → camera optical frame`.
- `all_methods` lists TSAI/PARK/HORAUD/DANIILIDIS; close agreement = good.

A converged run on this rig gave spread **3.8 mm**, relative-rotation median 34°,
`xyz [-0.0945, 0.0017, -0.0538]` — the URDF camera mount was ~4.6 cm off in z,
which the teach `grasp_offset` had been compensating.

## Gotchas

- **Vertical/overhead views fold the wrist into the lidar.** The orbit poses
  reach forward and are IK collision-filtered (the lidar is in the robot model),
  so the arm stays clear — but watch the run with an e-stop ready.
- Planning uses the **approximate URDF** `tcp→camera` only to aim the camera; the
  calibration measures the **true** transform from actual board + TCP poses, so a
  rough prior does not bias the result.
- Keep the board **rigidly fixed** during the whole run.
- The legacy `scripts/handeye_calibrate.py` (fixed-pose wrist sweep) is kept for
  reference but does **not** converge on this rig — use the orbit script.
