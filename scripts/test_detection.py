#!/usr/bin/env python3
"""Test Grounding DINO detection end-to-end.

Grabs one frame from the wrist camera via ROS 2, sends it to the remote
detection server, draws bounding boxes + centres on the result, and saves
the annotated image to disk.

Usage:
    python3 scripts/test_detection.py
    python3 scripts/test_detection.py --prompt "cup" --output /tmp/out.jpg
    python3 scripts/test_detection.py --image /tmp/existing.jpg --prompt "bottle"
    python3 scripts/test_detection.py --server http://100.112.100.20:30543
"""

import argparse
import base64
import json
import os
import sys
import urllib.request

import cv2


# ── defaults (mirror detection_params.yaml) ─────────────────────────────────
DEFAULT_SERVER = "http://100.112.100.20:30543"
DEFAULT_PROMPT = "bottle"
DEFAULT_TOPIC = "/piper/wrist_camera/piper_d405/color/image_rect_raw"
DEFAULT_OUTPUT = os.path.join(os.path.dirname(__file__), "..", "detection_annotated.jpg")
DEFAULT_BOX_THRESHOLD = 0.35
DEFAULT_TEXT_THRESHOLD = 0.25
DEFAULT_MAX_DETECTIONS = 5
JPEG_QUALITY = 70
FRAME_GRAB_TIMEOUT_SEC = 5.0


# ── colours (BGR) ────────────────────────────────────────────────────────────
COLOURS = [
    (0, 255, 0),    # green
    (255, 128, 0),  # orange
    (0, 200, 255),  # cyan
    (255, 0, 255),  # magenta
    (255, 255, 0),  # yellow
]


def grab_ros_frame(topic: str, timeout_sec: float) -> "cv2.Mat":
    """Subscribe to a ROS 2 image topic and return the first frame as BGR."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import time

    class _Grabber(Node):
        def __init__(self):
            super().__init__("detection_test_grabber")
            self.bridge = CvBridge()
            self.frame = None
            self.create_subscription(Image, topic, self._cb, 1)

        def _cb(self, msg):
            if self.frame is None:
                self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    rclpy.init()
    node = _Grabber()
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline and node.frame is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    frame = node.frame
    node.destroy_node()
    rclpy.shutdown()

    if frame is None:
        raise RuntimeError(
            f"No frame received on '{topic}' within {timeout_sec}s. "
            "Is the camera running?"
        )
    return frame


def encode_jpeg(img, quality: int = JPEG_QUALITY) -> bytes:
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError("Failed to JPEG-encode image")
    return buf.tobytes()


def call_detect(server: str, img_bytes: bytes, prompt: str,
                box_threshold: float, text_threshold: float,
                max_detections: int) -> dict:
    payload = json.dumps({
        "image": base64.b64encode(img_bytes).decode(),
        "prompt": prompt,
        "box_threshold": box_threshold,
        "text_threshold": text_threshold,
        "max_detections": max_detections,
    }).encode()
    req = urllib.request.Request(
        f"{server}/detect",
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=15) as resp:
        return json.loads(resp.read())


def annotate(img, detections: list) -> "cv2.Mat":
    out = img.copy()
    h, w = out.shape[:2]

    for i, det in enumerate(detections):
        colour = COLOURS[i % len(COLOURS)]
        cx = int(det["cx"])
        cy = int(det["cy"])
        bw = int(det["w"])
        bh = int(det["h"])
        x1, y1 = cx - bw // 2, cy - bh // 2
        x2, y2 = cx + bw // 2, cy + bh // 2

        # bounding box
        cv2.rectangle(out, (x1, y1), (x2, y2), colour, 2)

        # centre marker
        cv2.circle(out, (cx, cy), 5, (0, 0, 255), -1)
        cv2.drawMarker(out, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 16, 2)

        # label above box
        label = f"{det['class_id']} {det['score']:.2f}"
        (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        label_y = max(y1 - 6, lh + 4)
        cv2.putText(out, label, (x1, label_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)

        # coordinates below box
        coord = f"cx={cx} cy={cy}  {bw}x{bh}px"
        cv2.putText(out, coord, (x1, min(y2 + 18, h - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)

    # summary banner
    summary = f"prompt: \"{detections[0]['class_id'] if detections else '?'}\"  " \
              f"detections: {len(detections)}"
    cv2.putText(out, summary, (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1,
                cv2.LINE_AA)
    return out


def main():
    parser = argparse.ArgumentParser(description="Test Grounding DINO detection with bounding-box overlay")
    parser.add_argument("--server", default=DEFAULT_SERVER,
                        help=f"Detection server URL (default: {DEFAULT_SERVER})")
    parser.add_argument("--prompt", default=DEFAULT_PROMPT,
                        help=f"Detection prompt (default: '{DEFAULT_PROMPT}')")
    parser.add_argument("--image", default=None,
                        help="Path to an existing image file (skips ROS camera grab)")
    parser.add_argument("--topic", default=DEFAULT_TOPIC,
                        help=f"ROS 2 image topic (default: {DEFAULT_TOPIC})")
    parser.add_argument("--output", default=DEFAULT_OUTPUT,
                        help=f"Output annotated image path (default: {DEFAULT_OUTPUT})")
    parser.add_argument("--box-threshold", type=float, default=DEFAULT_BOX_THRESHOLD)
    parser.add_argument("--text-threshold", type=float, default=DEFAULT_TEXT_THRESHOLD)
    parser.add_argument("--max-detections", type=int, default=DEFAULT_MAX_DETECTIONS)
    args = parser.parse_args()

    # ── 1. Health check ──────────────────────────────────────────────────────
    print(f"[1/4] Checking server health at {args.server} ...")
    try:
        with urllib.request.urlopen(f"{args.server}/health", timeout=5) as r:
            health = json.loads(r.read())
        if health.get("status") != "ok" or not health.get("model_loaded"):
            print(f"  WARNING: server not fully ready: {health}")
        else:
            print(f"  OK  model_loaded={health['model_loaded']}")
    except Exception as e:
        print(f"  ERROR: {e}")
        sys.exit(1)

    # ── 2. Get image ─────────────────────────────────────────────────────────
    if args.image:
        print(f"[2/4] Loading image from {args.image} ...")
        img = cv2.imread(args.image)
        if img is None:
            print(f"  ERROR: could not read {args.image}")
            sys.exit(1)
    else:
        print(f"[2/4] Grabbing frame from ROS topic '{args.topic}' ...")
        try:
            img = grab_ros_frame(args.topic, FRAME_GRAB_TIMEOUT_SEC)
        except Exception as e:
            print(f"  ERROR: {e}")
            sys.exit(1)
    print(f"  Image shape: {img.shape[1]}x{img.shape[0]}")

    # ── 3. Detect ────────────────────────────────────────────────────────────
    print(f"[3/4] Sending to detector with prompt='{args.prompt}' ...")
    img_bytes = encode_jpeg(img)
    try:
        result = call_detect(
            args.server, img_bytes, args.prompt,
            args.box_threshold, args.text_threshold, args.max_detections
        )
    except Exception as e:
        print(f"  ERROR: {e}")
        sys.exit(1)

    print(f"  inference_ms : {result.get('inference_ms', '?'):.1f}")
    print(f"  num_detections: {result.get('num_detections', 0)}")
    for i, d in enumerate(result.get("detections", [])):
        print(f"  [{i}] {d['class_id']:20s}  score={d['score']:.3f}  "
              f"cx={d['cx']:.1f} cy={d['cy']:.1f}  "
              f"w={d['w']:.1f} h={d['h']:.1f}")

    # ── 4. Annotate & save ───────────────────────────────────────────────────
    print(f"[4/4] Annotating and saving to {args.output} ...")
    annotated = annotate(img, result.get("detections", []))
    output_path = os.path.abspath(args.output)
    cv2.imwrite(output_path, annotated)
    print(f"  Saved: {output_path}")


if __name__ == "__main__":
    main()
