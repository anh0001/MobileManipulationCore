#!/usr/bin/env python3
"""Integration test: client HTTP contract + Detection2DArray conversion."""

import json
import socket
import sys
import threading
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
from pathlib import Path

import numpy as np
import pytest
from std_msgs.msg import Header


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src" / "manipulation_detection"))

from manipulation_detection import remote_detection_client  # noqa: E402


class _MockDetectHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):  # noqa: A003
        return

    def do_POST(self):  # noqa: N802
        if self.path != "/detect":
            self.send_response(404)
            self.end_headers()
            return

        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length)
        request_payload = json.loads(body.decode("utf-8"))
        assert request_payload["prompt"] == "bottle"
        assert request_payload["image_encoding"] == "jpeg"

        response = {
            "detections": [
                {
                    "class_id": "bottle",
                    "score": 0.95,
                    "cx": 200.0,
                    "cy": 120.0,
                    "w": 60.0,
                    "h": 90.0,
                }
            ],
            "inference_ms": 43.2,
        }
        encoded = json.dumps(response).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)


def _pick_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


@pytest.mark.skipif(remote_detection_client.cv2 is None, reason="cv2 unavailable")
def test_mock_server_contract_and_header_consistency():
    port = _pick_free_port()
    server = ThreadingHTTPServer(("127.0.0.1", port), _MockDetectHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    try:
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        image_b64, width, height = remote_detection_client._encode_image_payload(  # pylint: disable=protected-access
            frame,
            jpeg_quality=70,
            max_long_side_px=320,
        )
        response = remote_detection_client._post_json(  # pylint: disable=protected-access
            f"http://127.0.0.1:{port}/detect",
            {
                "image": image_b64,
                "image_encoding": "jpeg",
                "image_width": width,
                "image_height": height,
                "prompt": "bottle",
                "stamp": {"sec": 11, "nanosec": 22},
                "frame_id": "camera_optical",
            },
            timeout_sec=0.5,
        )

        header = Header()
        header.frame_id = "camera_optical"
        header.stamp.sec = 11
        header.stamp.nanosec = 22
        detections_msg = remote_detection_client._build_detection_message(  # pylint: disable=protected-access
            header=header,
            detections=response.get("detections", []),
            min_score=0.3,
            max_detections=5,
        )

        assert detections_msg.header.frame_id == "camera_optical"
        assert detections_msg.header.stamp.sec == 11
        assert detections_msg.header.stamp.nanosec == 22
        assert len(detections_msg.detections) == 1
        assert detections_msg.detections[0].results[0].hypothesis.class_id == "bottle"
    finally:
        server.shutdown()
        server.server_close()
