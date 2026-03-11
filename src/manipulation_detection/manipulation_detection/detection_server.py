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

"""HTTP Grounding DINO detector service for visual-servo detections."""

import argparse
import base64
import errno
import io
import json
import logging
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
from typing import Any, Dict, List, Tuple

try:  # Optional runtime dependencies for remote detector host.
    import torch
    from PIL import Image
    from transformers import AutoModelForZeroShotObjectDetection
    from transformers import AutoProcessor
except ImportError:  # pragma: no cover - optional dependency
    torch = None
    Image = None
    AutoModelForZeroShotObjectDetection = None
    AutoProcessor = None


_DETECTOR_LOCK = threading.Lock()
_DETECTOR_MODEL = None
_DETECTOR_PROCESSOR = None
_DETECTOR_DEVICE = None
_CLIENT_DISCONNECT_ERRNOS = {
    errno.EPIPE,
    errno.ECONNRESET,
    errno.ECONNABORTED,
}


def _safe_float(value: Any, default: float) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(parsed):
        return default
    return parsed


def _safe_int(value: Any, default: int) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return default
    return parsed


def _resolve_prompt(request: Dict[str, Any]) -> str:
    prompt = request.get("prompt")
    if prompt is None:
        prompt = os.getenv("GROUNDING_DINO_DEFAULT_PROMPT", "")
    prompt = str(prompt).strip()
    if not prompt:
        raise ValueError("Request is missing non-empty 'prompt'.")
    if not prompt.endswith("."):
        prompt = f"{prompt}."
    return prompt


def _resolve_thresholds(request: Dict[str, Any]) -> Tuple[float, float, int]:
    default_box = _safe_float(os.getenv("GROUNDING_DINO_BOX_THRESHOLD", "0.35"), 0.35)
    default_text = _safe_float(os.getenv("GROUNDING_DINO_TEXT_THRESHOLD", "0.25"), 0.25)
    default_max = _safe_int(os.getenv("GROUNDING_DINO_MAX_DETECTIONS", "5"), 5)

    box_threshold = _safe_float(request.get("box_threshold"), default_box)
    text_threshold = _safe_float(request.get("text_threshold"), default_text)
    max_detections = _safe_int(request.get("max_detections"), default_max)

    box_threshold = max(0.0, min(1.0, box_threshold))
    text_threshold = max(0.0, min(1.0, text_threshold))
    max_detections = max(1, max_detections)
    return box_threshold, text_threshold, max_detections


def _decode_image(request: Dict[str, Any]):
    image_b64 = request.get("image")
    if not image_b64:
        raise ValueError("Request is missing 'image' field.")
    if Image is None:
        raise RuntimeError("Pillow is required for decoding detector input images.")

    try:
        image_bytes = base64.b64decode(image_b64)
    except Exception as exc:  # pragma: no cover - defensive
        raise ValueError(f"Invalid base64 image payload: {exc}") from exc

    try:
        image = Image.open(io.BytesIO(image_bytes))
        return image.convert("RGB")
    except Exception as exc:  # pragma: no cover - defensive
        raise ValueError(f"Failed to decode image payload: {exc}") from exc


def _load_detector():
    global _DETECTOR_MODEL, _DETECTOR_PROCESSOR, _DETECTOR_DEVICE

    if _DETECTOR_MODEL is not None and _DETECTOR_PROCESSOR is not None:
        return _DETECTOR_MODEL, _DETECTOR_PROCESSOR, _DETECTOR_DEVICE

    if (
        torch is None
        or Image is None
        or AutoProcessor is None
        or AutoModelForZeroShotObjectDetection is None
    ):
        raise RuntimeError(
            "Grounding DINO dependencies missing. Install torch, transformers, and pillow."
        )

    with _DETECTOR_LOCK:
        if _DETECTOR_MODEL is not None and _DETECTOR_PROCESSOR is not None:
            return _DETECTOR_MODEL, _DETECTOR_PROCESSOR, _DETECTOR_DEVICE

        model_id = os.getenv("GROUNDING_DINO_MODEL_ID", "IDEA-Research/grounding-dino-base")
        requested_device = os.getenv("GROUNDING_DINO_DEVICE")
        if requested_device:
            device = requested_device
        else:
            device = "cuda" if torch.cuda.is_available() else "cpu"

        dtype = torch.float32

        processor = AutoProcessor.from_pretrained(model_id)
        model = AutoModelForZeroShotObjectDetection.from_pretrained(
            model_id,
            torch_dtype=dtype,
        )
        model.to(device)
        model.eval()

        _DETECTOR_MODEL = model
        _DETECTOR_PROCESSOR = processor
        _DETECTOR_DEVICE = device
        logging.info("Loaded Grounding DINO model '%s' on %s", model_id, device)
        return _DETECTOR_MODEL, _DETECTOR_PROCESSOR, _DETECTOR_DEVICE


def _normalize_detections(
    boxes,
    scores,
    labels,
    max_detections: int,
) -> List[Dict[str, Any]]:
    detections: List[Dict[str, Any]] = []
    for box, score, label in zip(boxes, scores, labels):
        x1, y1, x2, y2 = [float(v) for v in box]
        if not all(math.isfinite(v) for v in [x1, y1, x2, y2]):
            continue
        width = max(0.0, x2 - x1)
        height = max(0.0, y2 - y1)
        if width <= 1e-6 or height <= 1e-6:
            continue
        cx = x1 + width * 0.5
        cy = y1 + height * 0.5
        detections.append(
            {
                "class_id": str(label),
                "score": float(score),
                "cx": cx,
                "cy": cy,
                "w": width,
                "h": height,
            }
        )

    detections.sort(key=lambda item: item["score"], reverse=True)
    return detections[:max_detections]


def run_detection(request: Dict[str, Any]) -> Dict[str, Any]:
    prompt = _resolve_prompt(request)
    box_threshold, text_threshold, max_detections = _resolve_thresholds(request)

    image = _decode_image(request)
    model, processor, device = _load_detector()

    target_size = [(image.size[1], image.size[0])]
    start = time.monotonic()
    inputs = processor(images=image, text=prompt, return_tensors="pt")
    inputs = {name: value.to(device) for name, value in inputs.items()}

    with torch.inference_mode():
        outputs = model(**inputs)

    processed = processor.post_process_grounded_object_detection(
        outputs=outputs,
        input_ids=inputs["input_ids"],
        target_sizes=target_size,
        box_threshold=box_threshold,
        text_threshold=text_threshold,
    )

    inference_ms = (time.monotonic() - start) * 1000.0
    raw = processed[0] if processed else {}
    boxes = raw.get("boxes", [])
    scores = raw.get("scores", [])
    labels = raw.get("labels", [])
    detections = _normalize_detections(boxes, scores, labels, max_detections=max_detections)

    return {
        "detections": detections,
        "inference_ms": float(inference_ms),
        "prompt": prompt,
        "num_detections": len(detections),
    }


class DetectionRequestHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, format, *args):  # noqa: A003 - BaseHTTPRequestHandler API
        logging.info("%s - %s", self.address_string(), format % args)

    def _send_json(self, payload: Dict[str, Any], status_code: int = 200):
        body = json.dumps(payload).encode("utf-8")
        try:
            self.send_response(status_code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except BrokenPipeError:
            self.close_connection = True
        except OSError as exc:
            if exc.errno in _CLIENT_DISCONNECT_ERRNOS:
                self.close_connection = True
                return
            raise

    def do_GET(self):  # noqa: N802 - BaseHTTPRequestHandler API
        if self.path != "/health":
            self._send_json({"error": "Not Found"}, status_code=404)
            return

        dependencies_ok = (
            torch is not None
            and Image is not None
            and AutoProcessor is not None
            and AutoModelForZeroShotObjectDetection is not None
        )
        self._send_json(
            {
                "status": "ok",
                "dependencies_ready": dependencies_ok,
                "model_loaded": _DETECTOR_MODEL is not None,
            }
        )

    def do_POST(self):  # noqa: N802 - BaseHTTPRequestHandler API
        if self.path != "/detect":
            self._send_json({"error": "Not Found"}, status_code=404)
            return

        length = self.headers.get("Content-Length")
        if length is None:
            self._send_json({"error": "Missing Content-Length"}, status_code=411)
            return

        try:
            size = int(length)
        except ValueError:
            self._send_json({"error": "Invalid Content-Length"}, status_code=400)
            return

        try:
            raw = self.rfile.read(size)
            request_payload = json.loads(raw.decode("utf-8"))
            if not isinstance(request_payload, dict):
                raise ValueError("JSON request body must be an object.")
        except json.JSONDecodeError as exc:
            self._send_json({"error": f"Invalid JSON: {exc}"}, status_code=400)
            return
        except ValueError as exc:
            self._send_json({"error": str(exc)}, status_code=400)
            return

        try:
            response = run_detection(request_payload)
        except ValueError as exc:
            self._send_json({"error": str(exc)}, status_code=400)
            return
        except Exception as exc:  # pragma: no cover - defensive
            logging.exception("Detection inference failed")
            self._send_json({"error": str(exc)}, status_code=500)
            return

        self._send_json(response, status_code=200)


def serve(host: str, port: int):
    server = ThreadingHTTPServer((host, port), DetectionRequestHandler)
    logging.info("Starting detection server on %s:%d", host, port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        logging.info("Shutting down detection server")
    finally:
        server.server_close()


def main(argv=None):
    parser = argparse.ArgumentParser(description="Grounding DINO remote detection server")
    parser.add_argument("--host", default="0.0.0.0", help="Host to bind")
    parser.add_argument("--port", type=int, default=30543, help="Port to bind")
    parser.add_argument(
        "--log-level",
        default="info",
        choices=["debug", "info", "warning", "error"],
        help="Log verbosity level",
    )
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="[%(asctime)s] %(levelname)s %(message)s",
    )
    serve(args.host, args.port)


if __name__ == "__main__":
    main()
