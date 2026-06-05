# Handover Skill

Hands a grasped object to a person. Uses the **base-mounted RealSense D435i**
(behind-and-right of the arm, looking forward) to find a person and their
distance, yaws the arm toward them, presents the object at a safe distance, and
opens the gripper so they can take it.

Skill: `manipulation_policy/skills/handover_skill.py` (name `handover`). Exposed
automatically as an MCP tool and over the `/execute_skill` action — no new node,
topic, or message. Run it after a successful `pick`.

## Why the D435i is grabbed on demand (not a ROS stream)

The D435i is a **free USB device** — only the wrist D405 runs as a
`realsense2_camera` node. Streaming the D435i continuously over ROS just to take
one snapshot is pure overhead. The skill instead opens the D435i **by serial**
with `pyrealsense2`, grabs **one aligned color+depth frame** on demand, and
closes it. No node, no topic, no continuous bandwidth, and no conflict (nothing
else owns the D435i).

## Pipeline

```
CHECK → CAPTURE → DETECT → LOCALIZE → (gates) → ORIENT → PRESENT → TAKE_OBJECT → RELEASE → RETRACT
```

1. **CHECK** — an object must be held (`gripper_width > held_width_threshold`),
   else abort. The visual-servo gates are cleared so it can't fight the arm.
2. **CAPTURE** — one aligned color+depth frame from the D435i (a few warm-up
   frames for auto-exposure, then `pipeline.stop()`).
3. **DETECT** — POST the color frame to the existing **Grounding DINO** HTTP
   service (`http://localhost:30543/detect`) with prompt `"person"`
   (open-vocabulary, already deployed — no new model). The dominant, central
   person is selected; if two are equally plausible and far apart, abort.
4. **LOCALIZE** — sample a **median depth over an upper-torso ROI**, deproject to
   a 3D point in the D435i optical frame, and map it to the arm-base XY plane via
   a **horizontal extrinsic** (camera position + yaw vs the arm base). This gives
   the person's **azimuth** and **distance**.
5. **Gates** — abort unless the person is within `[min_distance, max_distance]`
   and within `±max_azimuth`. (Too close ⇒ ask them to step back.)
6. **ORIENT → PRESENT** — `joint1` (base yaw) is set to the person's azimuth.
   The arm moves through a compact **staging** pose, then extends a **present**
   pose. Two short moves so the held object never sweeps sideways at full reach.
7. **TAKE_OBJECT → RELEASE** — dwell so the person can grasp it, then open the
   gripper (timed release — there is no force sensor).
8. **RETRACT** — return through staging to the ready pose.

## ⚠️ Calibrate before first real use

Two things must be set on the real robot (defaults are safe but not accurate):

### 1. D435i mount extrinsic (horizontal)

The D435i has no TF frame. v1 uses a **horizontal** extrinsic — enough to choose
a safe handover **direction**, not a full 6-DOF reach target. Optical frame is
`x=right, y=down, z=forward`; the skill uses `forward=z`, `left=-x`, rotates by
`camera_yaw`, and translates by `(camera_x, camera_y)` into the base frame.

Measure and set in `config/handover_params.yaml` (or live with `ros2 param set`):

```yaml
handover_camera_x_m:   -0.12   # camera behind the arm base (-X)
handover_camera_y_m:   -0.18   # camera right of the arm base (-Y)
handover_camera_yaw_rad: 0.0   # camera forward vs base +X (CCW +)
```

A quick check: stand a person straight ahead of the robot and confirm the
reported `azimuth_rad` (in the result JSON) is ≈ 0.

### 2. Taught staging / present poses

Hand-teach these 6-joint poses **at `joint1 = 0`** (facing straight forward);
`joint1` is overwritten at runtime with the person's azimuth.

```yaml
handover_staging_pose: [0.0, 1.2, -0.2, 0.0, -0.35, 0.0]   # compact / raised
handover_present_pose: [0.0, 1.2, -0.2, 0.0, -0.35, 0.0]   # reach forward + up  (TEACH)
```

They **default to the safe look-down ready pose**, so an untaught run only yaws
and holds — it will not fling the arm. Teach a real forward-reach `present_pose`
to actually extend the object toward the person. (Teach poses the same way as the
existing grasp poses, e.g. hand-guide the arm and read `/joint_states`.)

## Usage

```bash
# After a pick (object held):
ros2 action send_goal /execute_skill manipulation_msgs/action/ExecuteSkill \
  '{skill: "handover", params_json: "{}"}'

# Hold the object out a little longer before releasing:
ros2 action send_goal /execute_skill manipulation_msgs/action/ExecuteSkill \
  '{skill: "handover", params_json: "{\"dwell_sec\": 4.0}"}'
```

From an LLM/MCP client it is just the `handover` tool.

### Result JSON

```json
{
  "released": true, "distance_m": 1.39, "azimuth_rad": -0.13,
  "joint1_rad": -0.13, "depth_m": 1.50, "person_score": 0.91,
  "person_base_xy": [1.38, -0.18], "num_people": 1, "gripper_width": 0.07
}
```

## Key parameters

All under the `/skill_server` node (declared via the skill's `server_params`;
overridden by `config/handover_params.yaml`; settable live with `ros2 param set`).

| Parameter | Default | Meaning |
|---|---|---|
| `handover_camera_serial` | `243722070013` | D435i serial (`""` = first device) |
| `handover_detect_url` | `http://localhost:30543/detect` | Grounding DINO endpoint |
| `handover_min_distance_m` / `_max_distance_m` | `0.75` / `2.0` | safe handover distance band |
| `handover_max_azimuth_rad` | `1.0` | reject people too far to the side (~57°) |
| `handover_torso_frac` | `0.40` | depth sample point down the body |
| `handover_dwell_sec` | `2.5` | hold time before opening the gripper |
| `handover_require_object` | `true` | require a held object before handing over |
| `handover_camera_x/y_m`, `_yaw_rad` | see above | **CALIBRATE** mount extrinsic |
| `handover_staging_pose` / `_present_pose` | ready pose | **TEACH** present motion |

## Safety notes

- Aborts (no motion) if: nothing is held, no clear single person, the person is
  too close / too far / too far to the side, or the torso depth is invalid.
- A cancel during the dwell **keeps the object** (the gripper is not opened) and
  retracts.
- The release is **timed**, not force-triggered — there is no force/torque sensor
  and gripper-width "tug" detection is not reliable enough to be the default.
- Capture/detect lazily import `pyrealsense2`/`cv2`, so unit tests and CI run
  without a camera (see `tests/unit/test_handover_skill.py`).
```
