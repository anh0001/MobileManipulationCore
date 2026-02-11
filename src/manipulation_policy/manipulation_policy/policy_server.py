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

It runs OpenVLA inference and returns a PolicyOutput-compatible JSON response.
If no image is provided, the server returns a no-op policy output.
"""

import argparse
import base64
import errno
import io
import json
import logging
import math
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Iterable, Optional, Tuple

try:  # Optional runtime dependencies for OpenVLA inference.
    import torch
    from transformers import AutoModelForVision2Seq, AutoProcessor
    from PIL import Image
except ImportError:  # pragma: no cover - optional dependency
    torch = None
    AutoModelForVision2Seq = None
    AutoProcessor = None
    Image = None

try:  # Optional, only used for robust action conversion.
    import numpy as np
except ImportError:  # pragma: no cover - optional dependency
    np = None


_OPENVLA_LOCK = threading.Lock()
_OPENVLA_MODEL = None
_OPENVLA_PROCESSOR = None
_OPENVLA_DEVICE = None
_OPENVLA_DTYPE = None
_CLIENT_DISCONNECT_ERRNOS = {
    errno.EPIPE,
    errno.ECONNRESET,
    errno.ECONNABORTED,
}


def _env_flag(name: str, default: bool = False) -> bool:
    """Parse common boolean env-var values."""
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in ("1", "true", "yes", "on")


def _load_openvla() -> Tuple[Any, Any, str, Any]:
    """Load OpenVLA model and processor once (thread-safe)."""
    global _OPENVLA_MODEL, _OPENVLA_PROCESSOR, _OPENVLA_DEVICE, _OPENVLA_DTYPE

    if _OPENVLA_MODEL is not None and _OPENVLA_PROCESSOR is not None:
        return _OPENVLA_MODEL, _OPENVLA_PROCESSOR, _OPENVLA_DEVICE, _OPENVLA_DTYPE

    if torch is None or AutoProcessor is None or AutoModelForVision2Seq is None or Image is None:
        raise RuntimeError(
            "OpenVLA dependencies missing. Install torch, transformers, and pillow."
        )

    with _OPENVLA_LOCK:
        if _OPENVLA_MODEL is not None and _OPENVLA_PROCESSOR is not None:
            return _OPENVLA_MODEL, _OPENVLA_PROCESSOR, _OPENVLA_DEVICE, _OPENVLA_DTYPE

        model_id = os.getenv("OPENVLA_MODEL_ID", "openvla/openvla-7b")
        requested_device = os.getenv("OPENVLA_DEVICE")
        if requested_device:
            device = requested_device
        else:
            device = "cuda" if torch.cuda.is_available() else "cpu"

        if device.startswith("cuda"):
            use_bf16 = hasattr(torch.cuda, "is_bf16_supported") and torch.cuda.is_bf16_supported()
            dtype = torch.bfloat16 if use_bf16 else torch.float16
        else:
            dtype = torch.float32

        attn_impl = os.getenv("OPENVLA_ATTENTION_IMPL", "flash_attention_2")
        processor = AutoProcessor.from_pretrained(model_id, trust_remote_code=True)

        model_kwargs: Dict[str, Any] = {
            "torch_dtype": dtype,
            "low_cpu_mem_usage": True,
            "trust_remote_code": True,
        }
        if attn_impl:
            model_kwargs["attn_implementation"] = attn_impl
        if attn_impl and device.startswith("cuda"):
            # FlashAttention initialization expects CUDA placement up front.
            model_kwargs["device_map"] = {"": device}

        load_kwargs = dict(model_kwargs)

        try:
            model = AutoModelForVision2Seq.from_pretrained(model_id, **load_kwargs)
        except Exception as exc:
            if "device_map" in load_kwargs:
                logging.warning(
                    "OpenVLA model load failed with device_map=%s: %s. Retrying without device_map.",
                    load_kwargs["device_map"],
                    exc,
                )
                load_kwargs.pop("device_map", None)
                try:
                    model = AutoModelForVision2Seq.from_pretrained(model_id, **load_kwargs)
                except Exception as second_exc:
                    if "attn_implementation" in load_kwargs:
                        logging.warning(
                            "OpenVLA model load failed with attn impl '%s': %s. Retrying without it.",
                            attn_impl,
                            second_exc,
                        )
                        load_kwargs.pop("attn_implementation", None)
                        model = AutoModelForVision2Seq.from_pretrained(model_id, **load_kwargs)
                    else:
                        raise
            elif "attn_implementation" in load_kwargs:
                logging.warning(
                    "OpenVLA model load failed with attn impl '%s': %s. Retrying without it.",
                    attn_impl,
                    exc,
                )
                load_kwargs.pop("attn_implementation", None)
                model = AutoModelForVision2Seq.from_pretrained(model_id, **load_kwargs)
            else:
                raise

        if "device_map" not in load_kwargs:
            model.to(device)
        model.eval()

        _OPENVLA_MODEL = model
        _OPENVLA_PROCESSOR = processor
        _OPENVLA_DEVICE = device
        _OPENVLA_DTYPE = dtype

        logging.info("Loaded OpenVLA model %s on %s", model_id, device)

        return _OPENVLA_MODEL, _OPENVLA_PROCESSOR, _OPENVLA_DEVICE, _OPENVLA_DTYPE


def _decode_image(request: Dict[str, Any]) -> Optional[Any]:
    """Decode base64 JPEG/PNG image into a PIL Image."""
    image_b64 = request.get("image")
    if not image_b64:
        return None

    if Image is None:
        raise RuntimeError("Pillow not available for image decoding.")

    try:
        image_bytes = base64.b64decode(image_b64)
    except Exception as exc:  # pragma: no cover - defensive
        raise ValueError(f"Invalid base64 image data: {exc}") from exc

    try:
        image = Image.open(io.BytesIO(image_bytes))
        return image.convert("RGB")
    except Exception as exc:  # pragma: no cover - defensive
        raise ValueError(f"Failed to decode image bytes: {exc}") from exc


def _build_joint_state_context(joint_states: Optional[Dict[str, Any]]) -> str:
    if not joint_states:
        return ""

    names = joint_states.get("name") or []
    positions = joint_states.get("position") or []
    if not names or not positions:
        return ""

    pairs = []
    for name, pos in zip(names, positions):
        try:
            pairs.append(f"{name}={float(pos):.3f}")
        except (TypeError, ValueError):
            continue

    if not pairs:
        return ""

    return "Joint states: " + ", ".join(pairs) + "."


def _build_prompt(request: Dict[str, Any]) -> str:
    instruction = (
        request.get("task")
        or request.get("instruction")
        or request.get("task_text")
        or "Determine the next manipulation action."
    )
    if not instruction.lower().startswith("what action"):
        instruction = f"What action should the robot take to {instruction}?"
    if _env_flag("OPENVLA_INCLUDE_JOINT_STATES_IN_PROMPT", default=False):
        joint_context = _build_joint_state_context(request.get("joint_states"))
        if joint_context:
            instruction = f"{instruction}\n{joint_context}"

    return f"In: {instruction}\nOut:"


def _action_to_list(action: Any) -> Iterable[float]:
    if isinstance(action, (tuple, list)) and len(action) == 2 and not isinstance(action[0], (float, int)):
        action = action[0]
    if torch is not None and torch.is_tensor(action):
        return action.detach().cpu().flatten().tolist()
    if np is not None and isinstance(action, np.ndarray):
        return action.flatten().tolist()
    if hasattr(action, "tolist"):
        return action.tolist()
    if isinstance(action, (list, tuple)):
        return list(action)
    raise ValueError(f"Unsupported action output type: {type(action)}")


def _gripper_from_action(value: float) -> float:
    """Map OpenVLA gripper value to [0, 1]."""
    try:
        val = float(value)
    except (TypeError, ValueError):
        return 0.0

    if val < 0.0 or val > 1.0:
        val = (val + 1.0) * 0.5
    return max(0.0, min(1.0, val))


def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def _empty_response(reference_frame: str) -> Dict[str, Any]:
    return {
        "has_eef_target": False,
        "eef_target_pose": {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "has_joint_deltas": False,
        "joint_deltas": [],
        "gripper_command": 0.0,
        "gripper_active": False,
        "has_base_hint": False,
        "base_velocity_hint": {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        },
        "reference_frame": reference_frame,
        "confidence": 0.0,
    }


def build_stub_response(request: Dict[str, Any]) -> Dict[str, Any]:
    """Run OpenVLA inference and return a PolicyOutput-compatible response."""
    reference_frame = request.get("reference_frame") or "base_link"
    image = _decode_image(request)
    if image is None:
        logging.warning("OpenVLA request missing image; returning no-op response.")
        return _empty_response(reference_frame)

    model, processor, device, dtype = _load_openvla()

    prompt = _build_prompt(request)
    inputs = processor(text=prompt, images=image, return_tensors="pt")
    for key, value in list(inputs.items()):
        if torch is not None and torch.is_tensor(value):
            if value.dtype.is_floating_point:
                inputs[key] = value.to(device=device, dtype=dtype)
            else:
                inputs[key] = value.to(device=device)

    unnorm_key = os.getenv("OPENVLA_UNNORM_KEY", "bridge_orig")
    with torch.inference_mode():
        action = model.predict_action(**inputs, unnorm_key=unnorm_key, do_sample=False)

    action_values = list(_action_to_list(action))
    if len(action_values) < 7:
        raise ValueError(f"OpenVLA action has insufficient dims: {action_values}")

    # OpenVLA emits delta actions; downstream should interpret accordingly.
    dx, dy, dz, roll, pitch, yaw, gripper = action_values[:7]
    if not all(math.isfinite(v) for v in (dx, dy, dz, roll, pitch, yaw, gripper)):
        logging.warning("OpenVLA produced non-finite action values; returning no-op response.")
        return _empty_response(reference_frame)
    qx, qy, qz, qw = _euler_to_quaternion(roll, pitch, yaw)

    return {
        "has_eef_target": True,
        "eef_target_pose": {
            "position": {"x": float(dx), "y": float(dy), "z": float(dz)},
            "orientation": {"x": float(qx), "y": float(qy), "z": float(qz), "w": float(qw)},
        },
        "has_joint_deltas": False,
        "joint_deltas": [],
        "gripper_command": _gripper_from_action(gripper),
        "gripper_active": True,
        "has_base_hint": False,
        "base_velocity_hint": {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        },
        "reference_frame": reference_frame,
        "confidence": 1.0,
    }


class PolicyRequestHandler(BaseHTTPRequestHandler):
    """HTTP handler for policy inference requests."""

    server_version = "MobileManipulationPolicyServer/0.1"

    def _send_json(self, payload: Dict[str, Any], status_code: int = 200) -> None:
        response = json.dumps(payload).encode("utf-8")
        try:
            self.send_response(status_code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(response)))
            self.end_headers()
            self.wfile.write(response)
        except OSError as exc:
            if exc.errno in _CLIENT_DISCONNECT_ERRNOS:
                logging.debug("Client disconnected before response write completed: %s", exc)
                self.close_connection = True
                return
            raise

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
    parser.add_argument("--port", type=int, default=30542, help="Bind port")
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
