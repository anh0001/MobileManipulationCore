#!/usr/bin/env python3
"""Unit tests for remote_detection_client helpers."""

import base64
import sys
from pathlib import Path

import numpy as np
import pytest
from std_msgs.msg import Header


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src" / "manipulation_detection"))

from manipulation_detection import remote_detection_client  # noqa: E402


def test_build_detection_message_filters_by_score_and_preserves_header():
    header = Header()
    header.frame_id = "camera_frame"
    header.stamp.sec = 12
    header.stamp.nanosec = 34

    msg = remote_detection_client._build_detection_message(  # pylint: disable=protected-access
        header=header,
        detections=[
            {"class_id": "bottle", "score": 0.91, "cx": 320.0, "cy": 240.0, "w": 80.0, "h": 120.0},
            {"class_id": "cup", "score": 0.2, "cx": 100.0, "cy": 90.0, "w": 40.0, "h": 40.0},
        ],
        min_score=0.3,
        max_detections=5,
    )

    assert msg.header.frame_id == "camera_frame"
    assert msg.header.stamp.sec == 12
    assert msg.header.stamp.nanosec == 34
    assert len(msg.detections) == 1
    assert msg.detections[0].results[0].hypothesis.class_id == "bottle"
    assert msg.detections[0].results[0].hypothesis.score == pytest.approx(0.91)
    assert msg.detections[0].bbox.center.position.x == pytest.approx(320.0)
    assert msg.detections[0].bbox.center.position.y == pytest.approx(240.0)


def test_encode_image_payload_resizes_when_long_side_exceeds_limit():
    if remote_detection_client.cv2 is None:
        pytest.skip("cv2 unavailable in test environment")

    image = np.zeros((200, 100, 3), dtype=np.uint8)

    image_b64, width, height = remote_detection_client._encode_image_payload(  # pylint: disable=protected-access
        image,
        jpeg_quality=70,
        max_long_side_px=80,
    )

    decoded = base64.b64decode(image_b64)
    assert len(decoded) > 0
    assert max(width, height) <= 80
