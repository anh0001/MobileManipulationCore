# Look-then-move Table-plane Grasp

Top-down grasp pipeline for the PiPER arm + RealSense D405 wrist camera, driven
by Grounding DINO detections. Designed to be robust for **translucent objects**
(bottles) whose own surface depth is unreliable, and to respect the D405's
**~8.8 cm minimum depth** (it goes blind exactly where a continuous depth servo
would need it most).

Node: `manipulation_visual_servo/visual_servo_node`. Enabled with
`control_mode:=visual_servo arm_execution_mode:=move_group`.

## State machine

```
ACQUIRE → TRACK → ESTIMATE_GRASP → OPEN_GRIPPER → GUARDED_APPROACH → CLOSE_GRIPPER → LIFT → DONE
```

- **Capture pose (look-down):** the arm holds a wrist-down pose
  (`[0, 1.2, -0.2, 0, -0.35, 0]`, ~52° down, optical axis hitting the table
  ~0.43 m ahead) so the object is centred and the table is viewed steeply, not
  at a grazing angle. Set as `moveit.ready_pose_joint_positions` in
  `config/robot_params.yaml`.
- **ESTIMATE_GRASP** (`estimate_table_grasp` in `visual_servo_utils.cpp`):
  1. Fit the table plane from valid depth in a *shrunk annulus around* the bbox
     (excludes the object interior, so its bad depth is ignored).
  2. Footprint = robust **median of the object's own above-plane points**
     (15–300 mm above the plane) dropped onto the plane. Falls back to a
     bbox-bottom ray ∩ plane if too few object points are valid.
  3. Object top height = 90th-percentile of those point heights.
  4. Grasp point = footprint + table-normal × grasp height, transformed to the
     base frame, plus a fixed hand-eye `grasp_offset_{x,y,z}`.
  5. **Reach guard:** reject targets beyond `grasp_max_reach_m` (≈ PiPER reach).
- **GUARDED_APPROACH:** in `move_group` mode it publishes the *full* EEF delta;
  the adapter (move_group + delta mode) plans `current+delta` and executes a
  non-singular trajectory to the pre-grasp, then descends vertically to the
  grasp. EEF-TF feedback advances the legs (no depth servo, no blind push).

## Key parameters (`config/visual_servo_params.yaml`, `config/robot_params.yaml`)

| Param | Meaning |
|-------|---------|
| `grasp_use_move_group` | Skip servo-align; plan approach with MoveIt (recommended). |
| `grasp_top_down` | Grasp over the footprint and descend vertically. |
| `grasp_plane_annulus_frac` | Table-fit ring size around the bbox (0.3 = clean). |
| `grasp_plane_max_rms_m` | Reject a noisy (non-planar) table fit. |
| `grasp_height_above_table_m` | Grasp height when not using cap/neck reach. |
| `neck_grasp_offset_m` | If used, grasp this far below the detected object top. |
| `grasp_offset_{x,y,z}` | Fixed hand-eye calibration correction (base frame). |
| `pregrasp_standoff_m` | Pre-grasp distance above the grasp. |
| `grasp_max_reach_m` | Reach guard (horizontal distance from base). |
| `gripper_open_position` / `_tolerance` | Full-open target + how close before approaching. |
| `gripper_max_effort` | **Must be nonzero** or the PiPER gripper does not actuate. |

## Teach-based hand-eye calibration

The estimate carries a small residual offset (URDF camera placement vs the real
D405). Calibrate it by demonstration:

1. Run the grasp once and note `[ESTIMATE] grasp=(x,y,z)` for a fixed object.
2. Hand-guide the gripper (PiPER imitation/teach button) onto the correct grasp.
3. Record the true TCP: `ros2 run tf2_ros tf2_echo piper_base_link piper_tcp`.
4. Set `grasp_offset_{x,y,z} = true_TCP − estimated_grasp` (and pick a fixed
   `grasp_height_above_table_m` from the demo height).
5. **Power-cycle the arm** afterwards — the teach button latches `ctrl_mode=2`
   and the arm ignores position commands until reset.

## Gotchas learned on hardware

- **Gripper effort:** `GripperCommand.max_effort = 0` → the gripper reports the
  commanded joint value but does not physically move. Use a nonzero
  `gripper_max_effort`.
- **Do not cancel the gripper action goal on state change:** the PiPER releases
  its hold on cancel (opens then immediately closes; or drops on lift). Only
  cancel when aborting (LOST/IDLE); new goals preempt old ones.
- **Arm-only moves close the gripper (driver bug, FIXED in the fork):** the PiPER
  driver runs `GripperCtrl(joint_6)` on *every* command, and `joint_6` defaults to
  0 when the command carries < 7 joints — so a 6-DOF arm trajectory force-closes
  the gripper mid-approach (opens fully, then snaps shut ~1 s into the descent, so
  the hand arrives closed and misses). Fixed in `piper_ros` fork (branch `ranger`,
  c5ecaef): the FJT bridge now always publishes 7 joints, filling the gripper from
  its current measured angle so the driver holds it. Verified: gripper holds 0.069
  (full open) through the whole GUARDED_APPROACH, closes on the object, settles
  nonzero through LIFT. There is no MMC-side workaround (the arm controller rejects
  joint7 in arm goals), so this must stay fixed in the bridge.
- **Camera serial pinning:** with a D435i + D405 both attached, pin the wrist
  cam by serial (`wrist_camera_serial`) or it can grab the D405's topic after a
  USB re-enumeration; color/depth resolutions must also match (auto-pick can put
  color at 1280×720 vs depth 848×480, breaking the deprojection).
- **Gripper vs object width:** PiPER max opening ≈ 7 cm ≈ bottle body, so a
  body grasp has almost no tolerance — target the neck/cap or use custom
  fingertips.
- **Orphan cleanup:** killing the launch via SIGINT does not always kill the
  spawned nodes; stale `visual_servo_node`/`remote_detection_client` processes
  accumulate, load the Jetson, and starve the camera. Kill them by name between
  runs.

## Autonomous tuning (`scripts/grasp_autotune.py`)

A deterministic ROS 2 supervisor that closed-loop tunes the grasp without a human
labelling each attempt. Per attempt it resets the arm to the capture pose, sets
the tuning params on the running node via `ros2 param set` (the node re-reads
`grasp_offset_{x,y,z}`, `grasp_height_above_table_m`, `neck_grasp_offset_m`,
`grasp_enabled` in IDLE — no relaunch), gates one attempt with `grasp_enabled`,
records an MCAP bag, then classifies the outcome and updates the params.

Enablers in the node: `grasp_enabled` (IDLE gate), `grasp_auto_loop`
(DONE→IDLE so the next enable re-grasps), and runtime re-read of the tuning
params in IDLE.

Outcome classifier (wrist cam + gripper joint7 only, no force sensor):
- `gripper_did_not_open` — peak joint7 < open threshold during OPEN/approach.
- `gripper_closed_early` — joint7 closed before/at the descent (the MoveIt /
  controller-manager conflict; needs a code fix, the loop escalates it).
- `success` — joint7 settles NONZERO + stable after lift AND the object is gone
  from the table ROI.
- `miss_empty` — empty close, object still on the table → coordinate-descent
  nudge of `grasp_offset_x/y`.

Run: bringup (D405 pinned) + local DINO + grasp pipeline up, then the VSCode task
**"Run Grasp Autotune (autonomous)"** (or `python3 scripts/grasp_autotune.py`).
Results stream to `~/grasp_autotune.jsonl`, bags to `~/grasp_autotune_bags/`.
It stops on 3 successes, or escalates a repeated code-level failure to a human.

## Status

Verified live end-to-end: look-down capture, accurate table-plane estimate
(matches a teach demo within ~1 cm), MoveIt-planned approach without singularity
stalls, gripper held fully open through the entire descent (after the `piper_ros`
bridge fix above), close on the object, and lift with the object retained
(joint7 settles nonzero). Calibrated hand-eye offset
`grasp_offset = (-0.035, -0.018, 0.0)`, `grasp_height_above_table_m = 0.055`.
The `manipulation_adapter` continues to expose `moveit_servo` mode for the
image-based servo path.
