#!/usr/bin/env python3
"""Unit tests for policy server prompt construction."""

import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src" / "manipulation_policy"))

from manipulation_policy import policy_server  # noqa: E402


def _sample_request():
    return {
        "task": "pick up the bottle",
        "joint_states": {
            "name": ["joint1", "joint2"],
            "position": [0.1, -0.2],
        },
    }


def test_prompt_excludes_joint_state_context_by_default(monkeypatch):
    monkeypatch.delenv("OPENVLA_INCLUDE_JOINT_STATES_IN_PROMPT", raising=False)
    prompt = policy_server._build_prompt(_sample_request())
    assert "Joint states:" not in prompt
    assert "pick up the bottle" in prompt


def test_prompt_includes_joint_state_context_when_enabled(monkeypatch):
    monkeypatch.setenv("OPENVLA_INCLUDE_JOINT_STATES_IN_PROMPT", "true")
    prompt = policy_server._build_prompt(_sample_request())
    assert "Joint states:" in prompt
    assert "joint1=0.100" in prompt
