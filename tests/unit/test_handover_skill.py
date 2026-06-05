#!/usr/bin/env python3
"""Unit tests for the handover skill: geometry helpers + execute state machine.

Hardware I/O (D435i capture, HTTP detection) is monkeypatched, so these run with
no camera, no detector, and no ROS — pyrealsense2/cv2 are imported lazily inside
the patched methods and never touched here.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src" / "manipulation_policy"))

from manipulation_policy.skills.base import SkillContext  # noqa: E402
from manipulation_policy.skills.handover_skill import (  # noqa: E402
    HANDOVER_PARAM_DEFAULTS,
    HandoverSkill,
    _ambiguous,
    _clamp,
    _deproject,
    _depth_median_roi,
    _height_above_camera,
    _horizontal_forward,
    _optical_to_base,
    _select_people,
    _torso_pixel,
    _with_joint1,
)


# --------------------------------------------------------------------------- #
# Pure geometry / selection helpers
# --------------------------------------------------------------------------- #

def test_optical_to_base_forward_no_yaw():
    # Camera at origin, no yaw: forward (z) maps to base +X, left maps to base +Y.
    assert _optical_to_base(1.0, 0.0, 0.0, 0.0, 0.0) == pytest.approx((1.0, 0.0))
    assert _optical_to_base(0.0, 1.0, 0.0, 0.0, 0.0) == pytest.approx((0.0, 1.0))


def test_optical_to_base_yaw_90():
    # Yaw +90 deg: camera forward points to base +Y.
    px, py = _optical_to_base(1.0, 0.0, 0.0, 0.0, math.pi / 2)
    assert (px, py) == pytest.approx((0.0, 1.0), abs=1e-9)


def test_optical_to_base_translation():
    # Mount offset (behind-right) shifts the person point in the base frame.
    px, py = _optical_to_base(1.5, 0.0, -0.12, -0.18, 0.0)
    assert (px, py) == pytest.approx((1.38, -0.18))


def test_horizontal_forward_no_tilt_is_identity():
    assert _horizontal_forward(1.5, 0.3, 0.0) == pytest.approx(1.5)


def test_horizontal_forward_levels_downward_tilt():
    # Camera 5 deg down (roll=-0.0873) viewing a person at the same height, 1.5 m
    # ahead: the level ray sits 5 deg ABOVE the optical axis, so y_opt is up (-)
    # and z_opt is the slant range. Leveling must recover the true 1.5 m.
    roll = -0.0873
    z_opt = 1.5 * math.cos(roll)        # slant component along the optical axis
    y_opt = 1.5 * math.sin(roll)        # vertical component (negative = up)
    assert _horizontal_forward(z_opt, y_opt, roll) == pytest.approx(1.5, abs=1e-4)


def test_height_above_camera():
    # No tilt: a torso above the optical centre (y_opt negative = up) reads as
    # positive height above the camera; at centre height reads ~0.
    assert _height_above_camera(1.5, -0.40, 0.0) == pytest.approx(0.40)
    assert _height_above_camera(1.5, 0.0, 0.0) == pytest.approx(0.0)
    # A person at camera height, 1.5 m ahead, camera tilted 5 deg down -> ~0.
    roll = -0.0873
    z_opt = 1.5 * math.cos(roll)
    y_opt = 1.5 * math.sin(roll)
    assert _height_above_camera(z_opt, y_opt, roll) == pytest.approx(0.0, abs=1e-3)


def test_deproject_center_is_on_axis():
    intr = (600.0, 600.0, 320.0, 240.0, 640, 480)
    x, y, z = _deproject(320.0, 240.0, 1.5, intr)
    assert (x, y, z) == pytest.approx((0.0, 0.0, 1.5))
    # A pixel to the right deprojects to +X (right) in the optical frame.
    x2, _, _ = _deproject(440.0, 240.0, 1.5, intr)
    assert x2 == pytest.approx((440 - 320) / 600 * 1.5)


def test_depth_median_roi_basic():
    depth = np.full((480, 640), 1.5, dtype="float32")
    assert _depth_median_roi(depth, 320, 240, 10, 25, 0.20) == pytest.approx(1.5)


def test_depth_median_roi_rejects_sparse():
    depth = np.zeros((480, 640), dtype="float32")
    depth[240, 320] = 1.5  # a single valid pixel — below min_pixels
    assert _depth_median_roi(depth, 320, 240, 10, 25, 0.20) is None


def test_depth_median_roi_drops_holes():
    depth = np.zeros((480, 640), dtype="float32")
    depth[230:250, 310:330] = 1.2  # a solid valid patch among zeros
    assert _depth_median_roi(depth, 320, 240, 10, 25, 0.20) == pytest.approx(1.2)


def test_select_people_filters_and_sorts():
    dets = [
        {"cx": 100, "cy": 240, "w": 40, "h": 200, "score": 0.9, "class_id": "person"},   # ok small
        {"cx": 320, "cy": 240, "w": 120, "h": 320, "score": 0.9, "class_id": "person"},  # ok big
        {"cx": 500, "cy": 240, "w": 80, "h": 50, "score": 0.9, "class_id": "person"},    # short
        {"cx": 500, "cy": 240, "w": 80, "h": 200, "score": 0.1, "class_id": "person"},   # low
        {"cx": 500, "cy": 240, "w": 80, "h": 200, "score": 0.9, "class_id": "dog"},      # !person
    ]
    out = _select_people(dets, 640, 480, score_min=0.35, min_box_height_frac=0.20)
    assert len(out) == 2
    assert out[0]["w"] == 120  # largest first


def test_ambiguous_two_big_far_apart():
    people = [
        {"cx": 150, "cy": 240, "w": 120, "h": 300, "score": 0.9, "area": 120 * 300},
        {"cx": 500, "cy": 240, "w": 110, "h": 300, "score": 0.9, "area": 110 * 300},
    ]
    assert _ambiguous(people, 640, ratio=0.75) is True


def test_not_ambiguous_side_by_side():
    people = [
        {"cx": 300, "cy": 240, "w": 120, "h": 300, "score": 0.9, "area": 120 * 300},
        {"cx": 360, "cy": 240, "w": 110, "h": 300, "score": 0.9, "area": 110 * 300},
    ]
    assert _ambiguous(people, 640, ratio=0.75) is False  # close together = same direction


def test_with_joint1_overrides_index0():
    assert _with_joint1([0.0, 1.2, -0.2, 0.0, -0.35, 0.0], 0.5)[0] == 0.5
    assert _clamp(5.0, -2.5, 2.5) == 2.5


def test_torso_pixel_upper_body():
    person = {"cx": 320.0, "cy": 240.0, "w": 120.0, "h": 300.0}
    tx, ty = _torso_pixel(person, 0.40)
    assert tx == 320.0
    assert ty == pytest.approx(90.0 + 0.40 * 300.0)  # y_top=90


# --------------------------------------------------------------------------- #
# execute() state machine, with hardware I/O monkeypatched
# --------------------------------------------------------------------------- #

class _FakeCtx(SkillContext):
    def __init__(self, gripper_width=0.02, overrides=None):
        self._width = gripper_width
        self._now = 0.0
        self.arm_moves = []
        self.gripper_cmds = []
        self.prompts = []
        self.bool_params = []
        self.overrides = overrides or {}

    @property
    def vs_state(self):
        return "IDLE"

    @property
    def gripper_width(self):
        return self._width

    def get_param(self, name, default=None):
        if name in self.overrides:
            return self.overrides[name]
        extra = {
            "capture_pose": [0.0, 1.2, -0.2, 0.0, -0.35, 0.0],
            "held_width_threshold": 0.004,
            "gripper_open_position": 0.07,
            "gripper_max_effort": 5.0,
        }
        if name in extra:
            return extra[name]
        return HANDOVER_PARAM_DEFAULTS.get(name, default)

    def publish_prompt(self, text):
        self.prompts.append(text)

    def set_bool_param(self, name, value, node=None, timeout=4.0):
        self.bool_params.append((name, value))
        return True

    def set_double_array_param(self, name, values, node=None, timeout=4.0):
        return True

    def set_double_param(self, name, value, node=None, timeout=4.0):
        return True

    def get_remote_params(self, names, node=None, timeout=4.0):
        return {}

    def move_arm_to(self, positions, time_sec=5.0, timeout=12.0):
        self.arm_moves.append((list(positions), time_sec))
        return True

    def set_gripper(self, position, max_effort=5.0, timeout=10.0):
        self.gripper_cmds.append(position)
        self._width = position  # opening updates the measured width
        return True

    def log(self, message):
        pass

    def sleep(self, seconds):
        self._now += seconds

    def now(self):
        return self._now

    def ok(self):
        return True


def _person_det(cx=320, cy=240, w=120, h=320, score=0.9):
    return [{"cx": cx, "cy": cy, "w": w, "h": h, "score": score, "class_id": "person"}]


def _patch_io(skill, depth_value=1.5, dets=None, img=(640, 480)):
    """Monkeypatch capture + detection on the instance with a synthetic scene."""
    w, h = img
    intr = (600.0, 600.0, w / 2.0, h / 2.0, w, h)
    depth = np.full((h, w), float(depth_value), dtype="float32")
    color = np.zeros((h, w, 3), dtype="uint8")
    skill._capture_frame = lambda *a, **k: (color, depth, intr)
    detections = _person_det() if dets is None else dets
    skill._detect_person = lambda *a, **k: (detections, w, h)


def test_handover_happy_path():
    skill = HandoverSkill()
    _patch_io(skill, depth_value=1.5)  # person ~1.4 m ahead, slightly right
    ctx = _FakeCtx(gripper_width=0.02)
    res = skill.execute(ctx, {"dwell_sec": 0.2}, lambda *_a: None, lambda: False)

    assert res.success is True, res.message
    assert res.data["released"] is True
    assert res.data["distance_m"] == pytest.approx(1.39, abs=0.05)
    # staging, present, retract-staging, retract-ready = 4 moves
    assert len(ctx.arm_moves) == 4
    j1 = res.data["joint1_rad"]  # rounded to 4 dp in the result; moves use full precision
    assert ctx.arm_moves[0][0][0] == pytest.approx(j1, abs=1e-3)  # joint1 yawed toward person
    assert ctx.arm_moves[1][0][0] == pytest.approx(j1, abs=1e-3)
    assert ctx.gripper_cmds == [pytest.approx(0.07)]  # opened once to release


_LOW = [0.0, 1.0, -1.0, 0.0, 0.0, 0.0]
_HIGH = [0.0, 2.0, -2.0, 0.0, 1.0, 0.0]
# the default _patch_io scene yields head person_height ~0.36 m above the camera


def test_handover_present_full_high_when_tall():
    # Band entirely below the head height (0.36) => blend 1 => full HIGH pose.
    skill = HandoverSkill()
    _patch_io(skill, depth_value=1.5)
    ctx = _FakeCtx(gripper_width=0.02, overrides={
        "handover_person_height_low_m": 0.0, "handover_person_height_high_m": 0.3,
        "handover_present_pose": _LOW, "handover_present_pose_standing": _HIGH,
    })
    res = skill.execute(ctx, {"dwell_sec": 0.1}, lambda *_a: None, lambda: False)
    assert res.success is True, res.message
    assert res.data["present_blend"] == pytest.approx(1.0)
    # arm_moves: [0]=staging, [1]=present (joints 2-6 = HIGH pose).
    assert ctx.arm_moves[1][0][1:] == pytest.approx(_HIGH[1:])


def test_handover_present_full_low_when_short():
    # Band entirely above the head height => blend 0 => full LOW pose.
    skill = HandoverSkill()
    _patch_io(skill, depth_value=1.5)
    ctx = _FakeCtx(gripper_width=0.02, overrides={
        "handover_person_height_low_m": 0.4, "handover_person_height_high_m": 0.6,
        "handover_present_pose": _LOW, "handover_present_pose_standing": _HIGH,
    })
    res = skill.execute(ctx, {"dwell_sec": 0.1}, lambda *_a: None, lambda: False)
    assert res.success is True, res.message
    assert res.data["present_blend"] == pytest.approx(0.0)
    assert ctx.arm_moves[1][0][1:] == pytest.approx(_LOW[1:])


def test_handover_present_interpolates_midpoint():
    # head 0.36 in band [0.16, 0.56] => blend 0.5 => halfway between LOW and HIGH.
    skill = HandoverSkill()
    _patch_io(skill, depth_value=1.5)
    ctx = _FakeCtx(gripper_width=0.02, overrides={
        "handover_person_height_low_m": 0.16, "handover_person_height_high_m": 0.56,
        "handover_present_pose": _LOW, "handover_present_pose_standing": _HIGH,
    })
    res = skill.execute(ctx, {"dwell_sec": 0.1}, lambda *_a: None, lambda: False)
    assert res.success is True, res.message
    assert res.data["present_blend"] == pytest.approx(0.5, abs=0.02)
    mid = [(lo + hi) / 2 for lo, hi in zip(_LOW[1:], _HIGH[1:])]
    assert ctx.arm_moves[1][0][1:] == pytest.approx(mid, abs=0.02)


def test_handover_posture_override_forces_pose():
    # Explicit posture overrides the auto height blend regardless of detected height.
    ov = {
        "handover_person_height_low_m": 0.0, "handover_person_height_high_m": 0.5,
        "handover_present_pose": _LOW, "handover_present_pose_standing": _HIGH,
    }
    skill = HandoverSkill()
    _patch_io(skill, depth_value=1.5)
    ctx = _FakeCtx(gripper_width=0.02, overrides=ov)
    res = skill.execute(ctx, {"posture": "seated", "dwell_sec": 0.1},
                        lambda *_a: None, lambda: False)
    assert res.data["posture"] == "seated"
    assert res.data["present_blend"] == pytest.approx(0.0)
    assert ctx.arm_moves[1][0][1:] == pytest.approx(_LOW[1:])

    skill2 = HandoverSkill()
    _patch_io(skill2, depth_value=1.5)
    ctx2 = _FakeCtx(gripper_width=0.02, overrides=ov)
    res2 = skill2.execute(ctx2, {"posture": "standing", "dwell_sec": 0.1},
                          lambda *_a: None, lambda: False)
    assert res2.data["posture"] == "standing"
    assert res2.data["present_blend"] == pytest.approx(1.0)
    assert ctx2.arm_moves[1][0][1:] == pytest.approx(_HIGH[1:])


def test_height_blend_and_lerp():
    from manipulation_policy.skills.handover_skill import _height_blend, _lerp_pose
    assert _height_blend(0.0, 0.0, 0.6) == pytest.approx(0.0)
    assert _height_blend(0.6, 0.0, 0.6) == pytest.approx(1.0)
    assert _height_blend(0.3, 0.0, 0.6) == pytest.approx(0.5)
    assert _height_blend(1.0, 0.0, 0.6) == pytest.approx(1.0)   # clamped
    assert _height_blend(0.1, 0.5, 0.5) == pytest.approx(0.0)   # degenerate band
    assert _lerp_pose([0.0, 0.0], [2.0, 4.0], 0.5) == pytest.approx([1.0, 2.0])


def test_handover_aborts_without_object():
    skill = HandoverSkill()
    _patch_io(skill)
    ctx = _FakeCtx(gripper_width=0.0)  # nothing held
    res = skill.execute(ctx, {}, lambda *_a: None, lambda: False)
    assert res.success is False
    assert "no object held" in res.message
    assert ctx.arm_moves == []          # never moved
    assert ctx.gripper_cmds == []


def test_handover_aborts_no_person():
    skill = HandoverSkill()
    _patch_io(skill, dets=[])
    ctx = _FakeCtx()
    res = skill.execute(ctx, {}, lambda *_a: None, lambda: False)
    assert res.success is False
    assert "no person detected" in res.message
    assert ctx.arm_moves == []


def test_handover_aborts_too_close():
    skill = HandoverSkill()
    _patch_io(skill, depth_value=0.5)   # ~0.42 m from base < 0.75 m min
    ctx = _FakeCtx()
    res = skill.execute(ctx, {}, lambda *_a: None, lambda: False)
    assert res.success is False
    assert "too close" in res.message
    assert ctx.arm_moves == []          # gated before any motion
    assert ctx.gripper_cmds == []


def test_handover_aborts_bad_depth():
    skill = HandoverSkill()
    _patch_io(skill, depth_value=0.0)   # no valid depth anywhere
    ctx = _FakeCtx()
    res = skill.execute(ctx, {}, lambda *_a: None, lambda: False)
    assert res.success is False
    assert "no valid depth" in res.message
    assert ctx.arm_moves == []


def test_handover_aborts_ambiguous():
    skill = HandoverSkill()
    dets = [
        {"cx": 150, "cy": 240, "w": 120, "h": 320, "score": 0.9, "class_id": "person"},
        {"cx": 500, "cy": 240, "w": 115, "h": 320, "score": 0.9, "class_id": "person"},
    ]
    _patch_io(skill, dets=dets)
    ctx = _FakeCtx()
    res = skill.execute(ctx, {}, lambda *_a: None, lambda: False)
    assert res.success is False
    assert "equally plausible" in res.message
    assert ctx.arm_moves == []


def test_handover_cancel_during_dwell_keeps_object():
    skill = HandoverSkill()
    _patch_io(skill, depth_value=1.5)
    ctx = _FakeCtx(gripper_width=0.02)
    # is_cancelled: False for the after-capture and pre-present checks, then True
    # once the dwell starts polling (3rd call onward).
    calls = {"n": 0}

    def is_cancelled():
        calls["n"] += 1
        return calls["n"] >= 3

    res = skill.execute(ctx, {"dwell_sec": 2.0}, lambda *_a: None, is_cancelled)
    assert res.success is False
    assert "object kept" in res.message
    assert ctx.gripper_cmds == []       # gripper never opened -> object retained
    assert res.data["released"] is False


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
