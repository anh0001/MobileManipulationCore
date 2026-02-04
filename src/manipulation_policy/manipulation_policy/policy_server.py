#!/usr/bin/env python3
# Copyright 2026 MobileManipulationCore Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Lightweight HTTP policy server for remote inference.

This server exposes:
- GET /health
- POST /infer

It returns a stub PolicyOutput-compatible JSON response. Replace the
`build_stub_response` logic with real model inference when available.
"""

import argparse
import json
import logging
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict


def build_stub_response(request: Dict[str, Any]) -> Dict[str, Any]:
    """Build a stub policy response. Replace with real inference."""
    reference_frame = request.get("reference_frame") or "base_link"
    return {
        "has_eef_target": True,
        "eef_target_pose": {
            "position": {"x": 0.3, "y": 0.0, "z": 0.3},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "has_joint_deltas": False,
        "joint_deltas": [],
        "gripper_command": 0.5,
        "gripper_active": True,
        "has_base_hint": False,
        "base_velocity_hint": {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        },
        "reference_frame": reference_frame,
        "confidence": 0.0,
    }


class PolicyRequestHandler(BaseHTTPRequestHandler):
    """HTTP handler for policy inference requests."""

    server_version = "MobileManipulationPolicyServer/0.1"

    def _send_json(self, payload: Dict[str, Any], status_code: int = 200) -> None:
        response = json.dumps(payload).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(response)))
        self.end_headers()
        self.wfile.write(response)

    def do_GET(self) -> None:  # noqa: N802 (BaseHTTPRequestHandler naming)
        if self.path == "/health":
            self._send_json({"status": "ok"})
            return
        self._send_json({"error": "not found"}, status_code=404)

    def do_POST(self) -> None:  # noqa: N802 (BaseHTTPRequestHandler naming)
        if self.path != "/infer":
            self._send_json({"error": "not found"}, status_code=404)
            return

        content_length = int(self.headers.get("Content-Length", "0"))
        raw_body = self.rfile.read(content_length) if content_length > 0 else b""

        try:
            request_data = json.loads(raw_body.decode("utf-8")) if raw_body else {}
        except json.JSONDecodeError:
            self._send_json({"error": "invalid json"}, status_code=400)
            return

        try:
            response = build_stub_response(request_data)
        except Exception as exc:  # pragma: no cover - defensive
            logging.exception("Failed to process inference request: %s", exc)
            self._send_json({"error": "internal error"}, status_code=500)
            return

        self._send_json(response)

    def log_message(self, format: str, *args: Any) -> None:
        logging.info("%s - %s", self.address_string(), format % args)


def main() -> None:
    parser = argparse.ArgumentParser(description="Remote policy inference server")
    parser.add_argument("--host", default="0.0.0.0", help="Bind host")
    parser.add_argument("--port", type=int, default=5000, help="Bind port")
    parser.add_argument(
        "--log-level",
        default="info",
        choices=["debug", "info", "warning", "error", "critical"],
        help="Logging level",
    )
    args = parser.parse_args()

    logging.basicConfig(level=getattr(logging, args.log_level.upper()))

    server = ThreadingHTTPServer((args.host, args.port), PolicyRequestHandler)
    logging.info("Policy server listening on %s:%d", args.host, args.port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        logging.info("Policy server shutting down")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
