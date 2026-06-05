#!/usr/bin/env python3
"""Unit tests for pick skill result classification."""

import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src" / "manipulation_policy"))

from manipulation_policy.skills.pick_skill import PickSkill  # noqa: E402
from manipulation_policy.skills.base import SkillContext  # noqa: E402


class _FakePickContext(SkillContext):
    def __init__(self, gripper_width):
        self._states = ["OPEN_GRIPPER", "DONE"]
        self._state_index = 0
        self._now = 0.0
        self._gripper_width = gripper_width
        self.params_set = []
        self.prompts = []

    @property
    def vs_state(self) -> str:
        return self._states[min(self._state_index, len(self._states) - 1)]

    @property
    def gripper_width(self) -> float:
        return self._gripper_width

    def get_param(self, name: str, default=None):
        values = {
            "default_timeout_sec": 60.0,
            "acquire_timeout_sec": 20.0,
            "held_width_threshold": 0.012,
            "reset_arm_each_pick": False,
            "capture_pose": [0.0, 1.2, -0.2, 0.0, -0.35, 0.0],
        }
        return values.get(name, default)

    def publish_prompt(self, text: str) -> None:
        self.prompts.append(text)

    def set_bool_param(self, name: str, value: bool, node=None, timeout: float = 4.0) -> bool:
        self.params_set.append((name, value, node, timeout))
        return True

    def move_arm_to(self, positions, time_sec: float = 5.0, timeout: float = 12.0) -> bool:
        return True

    def set_gripper(self, position: float, max_effort: float = 5.0, timeout: float = 10.0) -> bool:
        return True

    def log(self, message: str) -> None:
        pass

    def sleep(self, seconds: float) -> None:
        self._now += seconds
        if seconds < 1.0:
            self._state_index += 1

    def now(self) -> float:
        return self._now

    def ok(self) -> bool:
        return True


def _run_pick(width):
    skill = PickSkill()
    ctx = _FakePickContext(width)
    feedback = lambda _state, _progress=0.0: None
    return skill.execute(ctx, {"object": "banana", "timeout_sec": 10.0}, feedback, lambda: False)


def test_pick_reports_empty_close_as_failure():
    result = _run_pick(0.0)

    assert result.success is False
    assert result.data["object_held"] is False
    assert result.data["likely_empty_grasp"] is True
    assert result.data["visual_servo_done"] is True
    assert "likely empty grasp" in result.message


def test_pick_reports_blocked_jaw_as_success():
    result = _run_pick(0.02)

    assert result.success is True
    assert result.data["object_held"] is True
    assert result.data["likely_empty_grasp"] is False
    assert result.data["visual_servo_done"] is True
