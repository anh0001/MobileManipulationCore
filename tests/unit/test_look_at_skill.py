#!/usr/bin/env python3
"""Unit tests for the look_at skill.

These exercise the real skill code with a fake SkillContext: the fake captures
the TCP pose the skill hands to compute_ik, and the tests reconstruct the camera
optical pose from it (using the same static optical<-tcp transform) to assert the
wrist camera's +Z really points from the camera at the target. The skill's IK
service, TF and arm controller are all stubbed, so no ROS is needed.
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src" / "manipulation_policy"))

from manipulation_policy.skills.look_at_skill import (  # noqa: E402
    LookAtSkill, _quat_to_rotation, _rotation_to_quat, _make_tf, _unit,
    _look_at_rotation)
from manipulation_policy.skills.base import SkillContext  # noqa: E402


ARM_JOINTS = [f"piper_joint{i}" for i in range(1, 7)]

# A non-trivial static "tcp expressed in optical" transform (translation + a
# 90-deg-ish rotation), so the optical<->tcp conversion is genuinely exercised.
_OPT_TCP_TRANS = (0.012, -0.020, -0.050)
_OPT_TCP_QUAT = _rotation_to_quat(
    np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]))
_T_OPT_TCP = _make_tf(np.asarray(_OPT_TCP_TRANS, float),
                      _quat_to_rotation(_OPT_TCP_QUAT))


class _FakeLookAtContext(SkillContext):
    """Stubs every SkillContext call the look_at skill makes."""

    def __init__(self, cam_pos, ik_solution=None, ik_returns_none=False,
                 cur_arm=None, tf_fail=False, ik_available=True,
                 move_reaches=True):
        self._cam_pos = tuple(float(v) for v in cam_pos)
        # ik_solution: a dict joint->pos (same each call), or a list of dicts
        # (one per roll candidate, indexed by call order; last entry repeats).
        self._ik_solution = ik_solution
        self._ik_returns_none = ik_returns_none
        self._cur_arm = list(cur_arm) if cur_arm is not None else None
        self._tf_fail = tf_fail
        self._ik_available = ik_available
        self._move_reaches = move_reaches           # arm reaches the solution?
        self.ik_requests = []                       # captured (pos, quat, frame)
        self.moved_to = None                        # joints passed to move_arm_to

    # --- params ---
    def get_param(self, name, default=None):
        values = {
            "arm_base_frame": "piper_base_link",
            "arm_joints": ARM_JOINTS,
            "look_at_planning_group": "piper_arm",
            "look_at_eef_link": "piper_tcp",
            "look_at_optical_frame": "piper_camera_optical_frame",
            "look_at_max_reach_m": 1.2,
            "look_at_min_standoff_m": 0.10,
            "look_at_ik_timeout_sec": 1.0,
            "look_at_roll_candidates_deg": [0.0, 30.0, -30.0, 60.0, -60.0,
                                            90.0, -90.0, 180.0],
            "look_at_up_axis": [0.0, 0.0, 1.0],
            "look_at_max_joint_step_rad": 1.5,
        }
        return values.get(name, default)

    # --- TF ---
    def lookup_transform(self, target_frame, source_frame):
        if self._tf_fail:
            return None
        if (target_frame, source_frame) == ("piper_base_link",
                                            "piper_camera_optical_frame"):
            return (self._cam_pos, (0.0, 0.0, 0.0, 1.0))
        if (target_frame, source_frame) == ("piper_camera_optical_frame",
                                            "piper_tcp"):
            return (_OPT_TCP_TRANS, _OPT_TCP_QUAT)
        return None

    def transform_point(self, target_frame, source_frame, point):
        # Tests always pass the target already in the arm base frame.
        return tuple(float(v) for v in point)

    def current_joint_positions(self):
        if self._cur_arm is None:
            return None
        return {j: float(v) for j, v in zip(ARM_JOINTS, self._cur_arm)}

    def ik_available(self):
        return self._ik_available

    def compute_ik(self, group, eef_link, position, orientation_xyzw, frame_id,
                   timeout=1.0, avoid_collisions=True):
        self.ik_requests.append((tuple(position), tuple(orientation_xyzw),
                                 frame_id))
        if self._ik_returns_none:
            return None
        sols = self._ik_solution
        if isinstance(sols, list):
            idx = min(len(self.ik_requests) - 1, len(sols) - 1)
            sol = sols[idx]
        else:
            sol = sols
        return dict(sol) if sol is not None else None

    def move_arm_to(self, positions, time_sec=5.0, timeout=12.0):
        self.moved_to = list(positions)
        if self._move_reaches:                       # simulate the arm settling
            self._cur_arm = [float(p) for p in positions]
        return True

    # --- unused-but-abstract plumbing ---
    @property
    def vs_state(self):
        return ""

    @property
    def gripper_width(self):
        return 0.0

    def publish_prompt(self, text):
        pass

    def set_bool_param(self, name, value, node=None, timeout=4.0):
        return True

    def set_double_array_param(self, name, values, node=None, timeout=4.0):
        return True

    def set_double_param(self, name, value, node=None, timeout=4.0):
        return True

    def get_remote_params(self, names, node=None, timeout=4.0):
        return {}

    def set_gripper(self, position, max_effort=5.0, timeout=10.0):
        return True

    def log(self, message):
        pass

    def sleep(self, seconds):
        pass

    def now(self):
        return 0.0

    def ok(self):
        return True


def _recover_optical_pose(pos, quat):
    """Optical-frame pose in base, recovered from the TCP pose the skill solved.

    The skill computes T_base_tcp = T_base_opt @ T_opt_tcp, so
    T_base_opt = T_base_tcp @ inv(T_opt_tcp).
    """
    t_base_tcp = _make_tf(np.asarray(pos, float), _quat_to_rotation(quat))
    return t_base_tcp @ np.linalg.inv(_T_OPT_TCP)


def _run(ctx, position, standoff_m=0.0, frame="base_footprint"):
    skill = LookAtSkill()
    params = {"position": list(position), "frame": frame,
              "standoff_m": float(standoff_m), "time_sec": 4.0}
    return skill.execute(ctx, params, lambda *_a: None, lambda: False)


def test_orientation_only_aims_wrist_camera_at_target():
    cam = (0.30, 0.0, 0.40)
    target = (0.50, 0.0, 0.10)
    ctx = _FakeLookAtContext(cam, ik_solution={j: 0.0 for j in ARM_JOINTS},
                             cur_arm=[0.0] * 6)
    res = _run(ctx, target, standoff_m=0.0)

    assert res.success is True, res.message
    assert res.data["mode"] == "orientation_only"
    # camera stayed where it was
    assert np.allclose(res.data["camera_position"], cam, atol=1e-6)
    # the executed solution was the IK solution
    assert ctx.moved_to == [0.0] * 6

    # reconstruct the optical pose the skill asked IK for and check the aim
    pos, quat, frame = ctx.ik_requests[0]
    assert frame == "piper_base_link"
    t_base_opt = _recover_optical_pose(pos, quat)
    opt_origin = t_base_opt[:3, 3]
    opt_z = t_base_opt[:3, 2]
    assert np.allclose(opt_origin, cam, atol=1e-6)              # camera at C
    expected_z = _unit(np.asarray(target) - np.asarray(cam))
    assert np.allclose(opt_z, expected_z, atol=1e-6)           # +Z aims at target


def test_standoff_places_camera_at_distance_and_aims():
    cam = (0.30, 0.0, 0.50)
    target = (0.30, 0.0, 0.10)         # straight below the camera
    standoff = 0.25
    ctx = _FakeLookAtContext(cam, ik_solution={j: 0.0 for j in ARM_JOINTS},
                             cur_arm=[0.0] * 6)
    res = _run(ctx, target, standoff_m=standoff)

    assert res.success is True, res.message
    assert res.data["mode"] == "standoff"
    c_b = np.asarray(res.data["camera_position"])
    # camera moved to `standoff` metres from the target, along the line back
    # toward the original camera (i.e. straight above the target here)
    assert math.isclose(float(np.linalg.norm(c_b - np.asarray(target))),
                        standoff, abs_tol=1e-6)
    assert np.allclose(c_b, (0.30, 0.0, 0.35), atol=1e-6)

    pos, quat, _ = ctx.ik_requests[0]
    t_base_opt = _recover_optical_pose(pos, quat)
    assert np.allclose(t_base_opt[:3, 3], c_b, atol=1e-6)
    expected_z = _unit(np.asarray(target) - c_b)
    assert np.allclose(t_base_opt[:3, 2], expected_z, atol=1e-6)


def test_standoff_below_minimum_is_clamped():
    ctx = _FakeLookAtContext((0.3, 0.0, 0.5),
                             ik_solution={j: 0.0 for j in ARM_JOINTS},
                             cur_arm=[0.0] * 6)
    res = _run(ctx, (0.3, 0.0, 0.1), standoff_m=0.01)   # below min 0.10
    assert res.success is True, res.message
    assert math.isclose(res.data["standoff_m"], 0.10, abs_tol=1e-6)


def test_no_ik_solution_fails_cleanly():
    ctx = _FakeLookAtContext((0.3, 0.0, 0.4), ik_returns_none=True,
                             cur_arm=[0.0] * 6)
    res = _run(ctx, (0.5, 0.0, 0.1))
    assert res.success is False
    assert "look at" in res.message.lower()
    assert ctx.moved_to is None        # never commanded the arm


def test_large_joint_move_is_rejected():
    # Every roll returns the same far-from-current solution -> guard trips.
    ctx = _FakeLookAtContext((0.3, 0.0, 0.4),
                             ik_solution={j: 2.0 for j in ARM_JOINTS},
                             cur_arm=[0.0] * 6)
    res = _run(ctx, (0.5, 0.0, 0.1))
    assert res.success is False
    assert "large arm move" in res.message
    assert ctx.moved_to is None


def test_fails_closed_when_current_joints_unknown():
    # No /joint_states -> can't vet/verify the move -> fail closed (do NOT run
    # a large open-loop sweep).
    ctx = _FakeLookAtContext((0.3, 0.0, 0.4),
                             ik_solution={j: 2.0 for j in ARM_JOINTS},
                             cur_arm=None)
    res = _run(ctx, (0.5, 0.0, 0.1))
    assert res.success is False
    assert "joint state unavailable" in res.message
    assert ctx.moved_to is None
    assert not ctx.ik_requests          # bailed before IK


def test_ik_service_unavailable_fails_fast():
    # MoveIt down -> one ik_available() check, no per-roll IK attempts.
    ctx = _FakeLookAtContext((0.3, 0.0, 0.4), cur_arm=[0.0] * 6,
                             ik_available=False)
    res = _run(ctx, (0.5, 0.0, 0.1))
    assert res.success is False
    assert "move_group" in res.message
    assert not ctx.ik_requests
    assert ctx.moved_to is None


def test_reports_failure_when_arm_does_not_reach_pose():
    # move_arm_to "completes" but the arm doesn't settle at the solution.
    ctx = _FakeLookAtContext((0.3, 0.0, 0.4),
                             ik_solution={j: 0.3 for j in ARM_JOINTS},
                             cur_arm=[0.0] * 6, move_reaches=False)
    res = _run(ctx, (0.5, 0.0, 0.1))
    assert res.success is False
    assert "did not reach" in res.message
    assert ctx.moved_to == [0.3] * 6       # it did command the move
    assert res.data["reach_error_rad"] > 0.15


def test_picks_smallest_move_when_upright_not_comfortable():
    # roll 0 solves but with a biggish (within-guard) move; roll 30 is small ->
    # pick roll 30. (comfortable=0.5, guard=1.5)
    big = {j: 0.8 for j in ARM_JOINTS}     # roll 0: max step 0.8 (> comfortable)
    small = {j: 0.1 for j in ARM_JOINTS}   # roll 30: max step 0.1 (<= comfortable)
    ctx = _FakeLookAtContext((0.3, 0.0, 0.4), ik_solution=[big, small],
                             cur_arm=[0.0] * 6)
    res = _run(ctx, (0.5, 0.0, 0.1))
    assert res.success is True, res.message
    assert ctx.moved_to == [0.1] * 6
    assert res.data["roll_deg"] == 30.0


def test_tf_unavailable_fails():
    ctx = _FakeLookAtContext((0.3, 0.0, 0.4), tf_fail=True, cur_arm=[0.0] * 6)
    res = _run(ctx, (0.5, 0.0, 0.1))
    assert res.success is False
    assert "TF unavailable" in res.message


def test_implausible_target_fails():
    ctx = _FakeLookAtContext((0.3, 0.0, 0.4),
                             ik_solution={j: 0.0 for j in ARM_JOINTS},
                             cur_arm=[0.0] * 6)
    res = _run(ctx, (5.0, 0.0, 0.0))     # way past max_reach 1.2 m
    assert res.success is False
    assert "implausible" in res.message
    assert not ctx.ik_requests           # bailed before IK


def test_malformed_position_fails():
    ctx = _FakeLookAtContext((0.3, 0.0, 0.4), cur_arm=[0.0] * 6)
    skill = LookAtSkill()
    res = skill.execute(ctx, {"position": [0.1, 0.2], "frame": "base_footprint",
                              "standoff_m": 0.0, "time_sec": 4.0},
                        lambda *_a: None, lambda: False)
    assert res.success is False
    assert "must be [x, y, z]" in res.message


def test_look_at_rotation_is_orthonormal_and_forward():
    # pure-math guard on the basis construction (REP-104 optical: +Z forward)
    z_dir = np.array([1.0, 0.0, 0.0])
    r = _look_at_rotation(z_dir, np.array([0.0, 0.0, 1.0]))
    assert np.allclose(r @ r.T, np.eye(3), atol=1e-9)
    assert math.isclose(float(np.linalg.det(r)), 1.0, abs_tol=1e-9)
    assert np.allclose(r[:, 2], z_dir, atol=1e-9)              # +Z forward
    assert np.allclose(r[:, 0], [0.0, -1.0, 0.0], atol=1e-9)   # +X right (-Y base)
    assert np.allclose(r[:, 1], [0.0, 0.0, -1.0], atol=1e-9)   # +Y down (-Z base)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
