#!/usr/bin/env python3
"""Unit tests for detection_server helpers."""

import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src" / "manipulation_policy"))

from manipulation_policy import detection_server  # noqa: E402


def test_resolve_prompt_uses_request_and_appends_period(monkeypatch):
    monkeypatch.setenv("GROUNDING_DINO_DEFAULT_PROMPT", "fallback object")
    prompt = detection_server._resolve_prompt({"prompt": "red bottle"})  # pylint: disable=protected-access
    assert prompt == "red bottle."


def test_resolve_prompt_falls_back_to_env(monkeypatch):
    monkeypatch.setenv("GROUNDING_DINO_DEFAULT_PROMPT", "cardboard box")
    prompt = detection_server._resolve_prompt({})  # pylint: disable=protected-access
    assert prompt == "cardboard box."


def test_resolve_prompt_raises_when_empty(monkeypatch):
    monkeypatch.delenv("GROUNDING_DINO_DEFAULT_PROMPT", raising=False)
    with pytest.raises(ValueError):
        detection_server._resolve_prompt({})  # pylint: disable=protected-access


def test_resolve_thresholds_clamps_and_defaults(monkeypatch):
    monkeypatch.setenv("GROUNDING_DINO_BOX_THRESHOLD", "0.33")
    monkeypatch.setenv("GROUNDING_DINO_TEXT_THRESHOLD", "0.22")
    monkeypatch.setenv("GROUNDING_DINO_MAX_DETECTIONS", "7")
    box_threshold, text_threshold, max_detections = detection_server._resolve_thresholds(  # pylint: disable=protected-access
        {"box_threshold": 2.0, "text_threshold": -1.0, "max_detections": 0}
    )
    assert box_threshold == pytest.approx(1.0)
    assert text_threshold == pytest.approx(0.0)
    assert max_detections == 1


def test_normalize_detections_sorts_and_limits():
    detections = detection_server._normalize_detections(  # pylint: disable=protected-access
        boxes=[
            [10.0, 20.0, 30.0, 60.0],
            [100.0, 80.0, 140.0, 130.0],
        ],
        scores=[0.5, 0.9],
        labels=["bottle", "cup"],
        max_detections=1,
    )
    assert len(detections) == 1
    top = detections[0]
    assert top["class_id"] == "cup"
    assert top["score"] == pytest.approx(0.9)
    assert top["cx"] == pytest.approx(120.0)
    assert top["cy"] == pytest.approx(105.0)
    assert top["w"] == pytest.approx(40.0)
    assert top["h"] == pytest.approx(50.0)
