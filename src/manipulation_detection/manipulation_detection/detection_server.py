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
    import numpy as np
    import torch
    from PIL import Image
    from transformers import AutoModelForZeroShotObjectDetection
    from transformers import AutoProcessor
except ImportError:  # pragma: no cover - optional dependency
    np = None
    torch = None
    Image = None
    AutoModelForZeroShotObjectDetection = None
    AutoProcessor = None

try:  # Optional MobileSAM for box-prompted segmentation masks.
    from mobile_sam import sam_model_registry as _sam_registry
    from mobile_sam import SamPredictor as _SamPredictor
except ImportError:  # pragma: no cover - optional dependency
    _sam_registry = None
    _SamPredictor = None

try:  # Optional CLIP for masked-crop re-ranking / disambiguation.
    from transformers import CLIPModel as _CLIPModel
    from transformers import CLIPProcessor as _CLIPProcessor
except ImportError:  # pragma: no cover - optional dependency
    _CLIPModel = None
    _CLIPProcessor = None


_DETECTOR_LOCK = threading.Lock()
_DETECTOR_MODEL = None
_DETECTOR_PROCESSOR = None
_DETECTOR_DEVICE = None
_SEGMENTER_LOCK = threading.Lock()
_SEGMENTER = None
_CLIP_LOCK = threading.Lock()
_CLIP_MODEL = None
_CLIP_PROCESSOR = None
_CLIP_DEVICE = None
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


def _load_segmenter():
    """Lazily load MobileSAM (box-prompted segmentation). None if unavailable."""
    global _SEGMENTER
    if _SEGMENTER is not None:
        return _SEGMENTER
    if _sam_registry is None or _SamPredictor is None or torch is None:
        return None
    with _SEGMENTER_LOCK:
        if _SEGMENTER is not None:
            return _SEGMENTER
        ckpt = os.getenv("MOBILE_SAM_CHECKPOINT", "/weights/mobile_sam.pt")
        if not os.path.exists(ckpt):
            logging.warning("MobileSAM checkpoint not found at %s; masks disabled", ckpt)
            return None
        device = os.getenv("GROUNDING_DINO_DEVICE") or (
            "cuda" if torch.cuda.is_available() else "cpu")
        sam = _sam_registry["vit_t"](checkpoint=ckpt)
        sam.to(device)
        sam.eval()
        _SEGMENTER = _SamPredictor(sam)
        logging.info("Loaded MobileSAM (vit_t) on %s", device)
        return _SEGMENTER


def _encode_mask_png(mask) -> str:
    """Encode a HxW boolean/uint8 mask as a base64 PNG (mode L, 255=object)."""
    arr = (np.asarray(mask).astype("uint8")) * 255
    img = Image.fromarray(arr, mode="L")
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return base64.b64encode(buf.getvalue()).decode("ascii")


def _segment_detections(image, detections: List[Dict[str, Any]]) -> None:
    """Run MobileSAM per detection box; attach a base64 PNG mask in-place."""
    predictor = _load_segmenter()
    if predictor is None or not detections:
        return
    image_np = np.asarray(image)  # HxWx3 RGB uint8
    with _SEGMENTER_LOCK:
        predictor.set_image(image_np)
        for det in detections:
            x1 = det["cx"] - det["w"] * 0.5
            y1 = det["cy"] - det["h"] * 0.5
            x2 = det["cx"] + det["w"] * 0.5
            y2 = det["cy"] + det["h"] * 0.5
            box = np.array([x1, y1, x2, y2], dtype="float32")
            with torch.inference_mode():
                masks, scores, _ = predictor.predict(
                    box=box, multimask_output=False)
            det["mask_png"] = _encode_mask_png(masks[0])
            det["mask_score"] = float(scores[0])
            det["_mask_arr"] = np.asarray(masks[0]).astype(bool)  # for CLIP masked crop


def _load_clip():
    """Lazily load CLIP for crop re-ranking. Returns (None, None, None) if absent."""
    global _CLIP_MODEL, _CLIP_PROCESSOR, _CLIP_DEVICE
    if _CLIP_MODEL is not None:
        return _CLIP_MODEL, _CLIP_PROCESSOR, _CLIP_DEVICE
    if _CLIPModel is None or _CLIPProcessor is None or torch is None:
        return None, None, None
    with _CLIP_LOCK:
        if _CLIP_MODEL is not None:
            return _CLIP_MODEL, _CLIP_PROCESSOR, _CLIP_DEVICE
        model_id = os.getenv("CLIP_MODEL_ID", "openai/clip-vit-base-patch32")
        device = os.getenv("GROUNDING_DINO_DEVICE") or (
            "cuda" if torch.cuda.is_available() else "cpu")
        processor = _CLIPProcessor.from_pretrained(model_id)
        model = _CLIPModel.from_pretrained(model_id).to(device)
        model.eval()
        _CLIP_MODEL = model
        _CLIP_PROCESSOR = processor
        _CLIP_DEVICE = device
        logging.info("Loaded CLIP model '%s' on %s", model_id, device)
        return _CLIP_MODEL, _CLIP_PROCESSOR, _CLIP_DEVICE


def _rerank_clip(image, detections, candidate_labels, target_label, margin, min_score,
                 candidate_descriptions=None, elimination_max_other=0.55):
    """Re-rank DINO boxes by CLIP P(label | crop).

    Grounding DINO proposes boxes; CLIP decides which physical object each box is,
    so the wrong-but-confident DINO box no longer wins. Mutates `detections` in
    place: sets score = P(target | crop), class_id = CLIP argmax label, and sorts
    best-target-first. Returns True when the choice is AMBIGUOUS (best is not the
    target, or below min_score, or within `margin` of the runner-up) so the caller
    can decline to grasp instead of picking wrong.
    """
    model, processor, device = _load_clip()
    if model is None or not detections or not candidate_labels:
        return False
    labels = list(candidate_labels)
    desc_map = {}
    if candidate_descriptions and len(candidate_descriptions) == len(candidate_labels):
        desc_map = {lbl: str(d) for lbl, d in zip(candidate_labels, candidate_descriptions)
                    if str(d).strip()}
    if target_label and target_label not in labels:
        labels = [target_label] + labels
    tgt_idx = labels.index(target_label) if target_label in labels else 0
    # CLIP text: prefer the discriminative description, else a generic template.
    texts = [desc_map.get(lbl, f"a photo of a {lbl}") for lbl in labels]
    crops = []
    for det in detections:
        x1 = max(0, int(det["cx"] - det["w"] * 0.5))
        y1 = max(0, int(det["cy"] - det["h"] * 0.5))
        x2 = max(x1 + 1, int(det["cx"] + det["w"] * 0.5))
        y2 = max(y1 + 1, int(det["cy"] + det["h"] * 0.5))
        crop = image.crop((x1, y1, x2, y2))
        # Prefer the SAM-masked crop: white out the background so CLIP scores the
        # object itself, not neighbouring clutter (greatly improves discrimination
        # between similar adjacent toys).
        mask_arr = det.get("_mask_arr")
        if mask_arr is not None and np is not None:
            try:
                sub = np.asarray(mask_arr[y1:y2, x1:x2], dtype=bool)
                arr = np.asarray(crop).copy()
                if sub.shape[:2] == arr.shape[:2] and sub.any():
                    arr[~sub] = 255
                    crop = Image.fromarray(arr)
            except Exception:  # pragma: no cover - masking is best-effort
                pass
        crops.append(crop)
    with _CLIP_LOCK:
        inputs = processor(text=texts, images=crops, return_tensors="pt", padding=True)
        inputs = {name: value.to(device) for name, value in inputs.items()}
        with torch.inference_mode():
            out = model(**inputs)
        probs = out.logits_per_image.softmax(dim=1).detach().cpu().numpy()
    for det, prob in zip(detections, probs):
        amax = int(prob.argmax())
        # other_conf = how strongly the box matches the best NON-target object.
        other_conf = max(
            (float(prob[i]) for i, lbl in enumerate(labels) if lbl != target_label),
            default=0.0)
        det["dino_score"] = float(det.get("score", 0.0))
        det["clip_target_score"] = float(prob[tgt_idx])
        det["clip_argmax_label"] = labels[amax]
        det["clip_argmax_score"] = float(prob[amax])
        det["clip_other_conf"] = other_conf
        det["class_id"] = labels[amax]
        det["score"] = float(prob[tgt_idx])

    target_dets = [d for d in detections if d["clip_argmax_label"] == target_label]
    if target_dets:
        # Positive-ID path: CLIP recognizes the target directly. Ambiguity is
        # competition between DIFFERENT objects (not duplicate boxes on the target).
        best = max(target_dets, key=lambda d: d["clip_target_score"])
        other_best = max(
            (d["clip_target_score"] for d in detections
             if d["clip_argmax_label"] != target_label),
            default=0.0)
        detections.sort(key=lambda d: (d is not best, -d["clip_target_score"]))
        return bool(
            best["clip_target_score"] < min_score
            or (best["clip_target_score"] - other_best) < margin)

    # Elimination / odd-one-out: CLIP cannot positively name the target (e.g. a
    # plush bread it has no good concept for), but it CAN confidently identify the
    # other objects. The target is then the box that matches the known objects
    # LEAST — the odd one out. Accept only if that box is genuinely unrecognized
    # (low other_conf) and distinctly more so than the runner-up.
    best = min(detections, key=lambda d: d["clip_other_conf"])
    detections.sort(key=lambda d: (d is not best, d["clip_other_conf"]))
    runner_other = min(
        (d["clip_other_conf"] for d in detections if d is not best), default=1.0)
    best["class_id"] = target_label
    best["score"] = float(1.0 - best["clip_other_conf"])
    return bool(
        best["clip_other_conf"] > elimination_max_other
        or (runner_other - best["clip_other_conf"]) < margin)


_COLOR_RGB = {
    "brown": (120, 80, 55), "red": (190, 35, 35), "yellow": (225, 200, 50),
    "green": (50, 140, 55), "orange": (220, 130, 40), "white": (235, 235, 235),
    "black": (30, 30, 30), "purple": (120, 50, 140), "pink": (235, 150, 170),
    "blue": (45, 70, 180),
}


def _select_by_color(image, detections, candidate_labels, scene_colors, target_label,
                     color_max_dist=95.0, color_margin=25.0):
    """Pick the box whose dominant (SAM-masked) colour matches the target colour.

    Appearance models (DINO, CLIP) confuse a brown bread with a red apple by shape;
    colour is orthogonal and unambiguous. Each object has an expected colour name
    (scene_colors, aligned with candidate_labels). For every box we take the mean
    RGB of its masked pixels, find the nearest reference colour, and select the box
    whose nearest colour is the target's. Ambiguous if none matches, the match is
    too far, or a differently-coloured box is about as close.
    """
    if np is None or not detections or not scene_colors:
        return False
    colormap = {lbl: str(scene_colors[i]).strip().lower()
                for i, lbl in enumerate(candidate_labels) if i < len(scene_colors)}
    target_color = colormap.get(target_label)
    target_ref = _COLOR_RGB.get(target_color) if target_color else None
    if target_ref is None:
        return False  # no colour known for the target -> colour can't decide
    arr = np.asarray(image).astype("float32")
    target_np = np.asarray(target_ref, dtype="float32")
    refs = {lbl: _COLOR_RGB.get(c) for lbl, c in colormap.items()}
    scored = []
    for det in detections:
        mask = det.get("_mask_arr")
        if mask is not None and getattr(mask, "any", lambda: False)():
            pix = arr[mask]
        else:
            x1 = max(0, int(det["cx"] - det["w"] * 0.5))
            y1 = max(0, int(det["cy"] - det["h"] * 0.5))
            x2 = max(x1 + 1, int(det["cx"] + det["w"] * 0.5))
            y2 = max(y1 + 1, int(det["cy"] + det["h"] * 0.5))
            pix = arr[y1:y2, x1:x2].reshape(-1, 3)
        if pix.size == 0:
            continue
        mean_rgb = pix.reshape(-1, 3).mean(axis=0)
        dist_target = float(np.linalg.norm(mean_rgb - target_np))
        closest_lbl, closest_d = None, 1e9
        for lbl, ref in refs.items():
            if ref is None:
                continue
            d = float(np.linalg.norm(mean_rgb - np.asarray(ref, dtype="float32")))
            if d < closest_d:
                closest_d, closest_lbl = d, lbl
        det["color_dist_target"] = dist_target
        det["color_closest_label"] = closest_lbl
        det["color_mean_rgb"] = [round(float(v), 1) for v in mean_rgb]
        scored.append(det)
    if not scored:
        return True
    target_boxes = [d for d in scored if d["color_closest_label"] == target_label]
    if not target_boxes:
        detections.sort(key=lambda d: d.get("color_dist_target", 1e9))
        return True
    best = min(target_boxes, key=lambda d: d["color_dist_target"])
    detections.sort(key=lambda d: (d is not best, d.get("color_dist_target", 1e9)))
    best["class_id"] = target_label
    best["score"] = float(max(0.0, 1.0 - best["color_dist_target"] / 255.0))
    other_best = min(
        (d["color_dist_target"] for d in scored
         if d["color_closest_label"] != target_label), default=1e9)
    return bool(
        best["color_dist_target"] > color_max_dist
        or (other_best - best["color_dist_target"]) < color_margin)


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

    if request.get("return_masks") and detections:
        try:
            _segment_detections(image, detections)
        except Exception:  # pragma: no cover - masks are best-effort
            logging.exception("MobileSAM segmentation failed; returning boxes only")

    # Disambiguation: decide which proposed box is actually the requested object,
    # and flag ambiguity so the caller can decline rather than grasp wrong.
    # selector = "color" (dominant masked colour) or "clip" (vision-language).
    ambiguous = False
    candidate_labels = request.get("candidate_labels")
    selector = str(request.get("selector", "clip")).strip().lower()
    if request.get("clip_rerank") and isinstance(candidate_labels, list) \
            and candidate_labels and detections:
        labels_list = [str(c) for c in candidate_labels]
        target = str(request.get("target_label", "")).strip()
        try:
            if selector == "color" and isinstance(request.get("scene_colors"), list):
                ambiguous = _select_by_color(
                    image, detections, labels_list,
                    scene_colors=[str(c) for c in request.get("scene_colors")],
                    target_label=target,
                    color_max_dist=_safe_float(request.get("color_max_dist"), 95.0),
                    color_margin=_safe_float(request.get("color_margin"), 25.0),
                )
            else:
                descs = request.get("candidate_descriptions")
                ambiguous = _rerank_clip(
                    image, detections, candidate_labels=labels_list,
                    target_label=target,
                    margin=_safe_float(request.get("clip_margin"), 0.10),
                    min_score=_safe_float(request.get("clip_min_score"), 0.30),
                    candidate_descriptions=[str(d) for d in descs]
                    if isinstance(descs, list) else None,
                    elimination_max_other=_safe_float(
                        request.get("clip_elimination_max_other"), 0.55),
                )
        except Exception:  # pragma: no cover - selection is best-effort
            logging.exception("Disambiguation failed; returning DINO order")

    for det in detections:  # drop the numpy mask (not JSON serializable)
        det.pop("_mask_arr", None)

    return {
        "detections": detections,
        "ambiguous": bool(ambiguous),
        "inference_ms": float(inference_ms),
        "prompt": prompt,
        "num_detections": len(detections),
        "image_height": int(image.size[1]),
        "image_width": int(image.size[0]),
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
