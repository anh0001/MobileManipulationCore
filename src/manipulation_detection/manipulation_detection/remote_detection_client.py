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
ROS 2 client node that forwards camera frames to a remote detector service.

The node compresses RGB frames to JPEG, sends them to an HTTP `/detect` endpoint,
and republishes results as `vision_msgs/msg/Detection2DArray`.
"""

import base64
import base64
import json
import math
import threading
import time
from typing import Any, Dict, List, Optional, Tuple
from urllib import error as urlerror
from urllib import request as urlrequest

try:
    import numpy as np
except ImportError:  # pragma: no cover - optional dependency
    np = None

import rclpy
from cv_bridge import CvBridge
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose

try:
    import cv2
except ImportError:  # pragma: no cover - optional dependency
    cv2 = None


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(parsed):
        return default
    return parsed


def _safe_int(value: Any, default: int = 0) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return default
    return parsed


def _encode_image_payload(
    cv_image,
    jpeg_quality: int,
    max_long_side_px: int,
) -> Tuple[str, int, int]:
    """Encode BGR image to base64 JPEG with optional resize."""
    if cv2 is None:
        raise RuntimeError("OpenCV is required for remote detection image encoding.")

    processed = cv_image
    height, width = processed.shape[:2]
    if max_long_side_px > 0:
        long_side = max(height, width)
        if long_side > max_long_side_px:
            scale = float(max_long_side_px) / float(long_side)
            new_width = max(1, int(round(width * scale)))
            new_height = max(1, int(round(height * scale)))
            processed = cv2.resize(processed, (new_width, new_height), interpolation=cv2.INTER_AREA)
            height, width = processed.shape[:2]

    quality = max(1, min(100, int(jpeg_quality)))
    ok, encoded = cv2.imencode(".jpg", processed, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise RuntimeError("Failed to JPEG-encode image for remote detection request.")

    return base64.b64encode(encoded.tobytes()).decode("ascii"), width, height


def _build_detection_message(
    header,
    detections: List[Dict[str, Any]],
    min_score: float,
    max_detections: int,
) -> Detection2DArray:
    """Convert pixel-space detection dictionaries to Detection2DArray."""
    msg = Detection2DArray()
    msg.header = header

    filtered = []
    for item in detections:
        score = _safe_float(item.get("score"), 0.0)
        if score < min_score:
            continue
        filtered.append(item)

    filtered.sort(key=lambda item: _safe_float(item.get("score"), 0.0), reverse=True)
    if max_detections > 0:
        filtered = filtered[:max_detections]

    for idx, item in enumerate(filtered):
        det = Detection2D()
        det.header = header
        det.id = str(item.get("id", str(idx)))
        det.bbox.center.position.x = _safe_float(item.get("cx"), 0.0)
        det.bbox.center.position.y = _safe_float(item.get("cy"), 0.0)
        det.bbox.center.theta = 0.0
        det.bbox.size_x = max(0.0, _safe_float(item.get("w"), 0.0))
        det.bbox.size_y = max(0.0, _safe_float(item.get("h"), 0.0))

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = str(item.get("class_id", ""))
        hypothesis.hypothesis.score = _safe_float(item.get("score"), 0.0)
        det.results.append(hypothesis)
        msg.detections.append(det)

    return msg


def _rescale_detections_to_source_image(
    detections: List[Dict[str, Any]],
    source_width: int,
    source_height: int,
    detector_width: int,
    detector_height: int,
) -> List[Dict[str, Any]]:
    """Map detector pixel coordinates back to the source image pixel coordinates."""
    if (
        source_width <= 0
        or source_height <= 0
        or detector_width <= 0
        or detector_height <= 0
    ):
        return detections

    if source_width == detector_width and source_height == detector_height:
        return detections

    scale_x = float(source_width) / float(detector_width)
    scale_y = float(source_height) / float(detector_height)

    scaled: List[Dict[str, Any]] = []
    for item in detections:
        transformed = dict(item)
        transformed["cx"] = _safe_float(item.get("cx"), 0.0) * scale_x
        transformed["cy"] = _safe_float(item.get("cy"), 0.0) * scale_y
        transformed["w"] = _safe_float(item.get("w"), 0.0) * scale_x
        transformed["h"] = _safe_float(item.get("h"), 0.0) * scale_y
        scaled.append(transformed)
    return scaled


def _post_json(
    url: str,
    payload: Dict[str, Any],
    timeout_sec: float,
) -> Dict[str, Any]:
    """POST JSON and decode JSON response."""
    body = json.dumps(payload).encode("utf-8")
    request = urlrequest.Request(
        url,
        data=body,
        headers={"Content-Type": "application/json"},
    )
    with urlrequest.urlopen(request, timeout=timeout_sec) as response:
        if response.status != 200:
            raise urlerror.HTTPError(url, response.status, response.reason, response.headers, None)
        raw = response.read().decode("utf-8")
    parsed = json.loads(raw)
    if not isinstance(parsed, dict):
        raise ValueError("Remote detector response must be a JSON object.")
    return parsed


class RemoteDetectionClientNode(Node):
    """Jetson-side detection bridge from image topic to remote HTTP detector."""

    def __init__(self):
        super().__init__("remote_detection_client")

        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("detection_topic", "/manipulation/target_detections")
        self.declare_parameter("prompt_topic", "/visual_servo/target_prompt")
        self.declare_parameter("remote_url", "http://localhost:30543")
        self.declare_parameter("request_rate_hz", 4.0)
        self.declare_parameter("request_timeout_sec", 0.30)
        self.declare_parameter("retry_attempts", 0)
        self.declare_parameter("max_result_staleness_sec", 0.40)
        self.declare_parameter("jpeg_quality", 70)
        self.declare_parameter("max_image_long_side_px", 640)
        self.declare_parameter("default_prompt", "")
        self.declare_parameter("box_threshold", 0.35)
        self.declare_parameter("text_threshold", 0.25)
        self.declare_parameter("min_score", 0.35)
        self.declare_parameter("max_detections", 5)
        self.declare_parameter("metrics_log_interval_sec", 5.0)
        self.declare_parameter("request_masks", True)
        self.declare_parameter("mask_topic", "/manipulation/target_mask")
        # CLIP re-rank / disambiguation. When enabled, Grounding DINO is prompted
        # with the whole scene vocabulary (proposes every object), CLIP picks the
        # box that actually matches the requested object, and an ambiguous result
        # publishes no detection so the robot declines rather than grasping wrong.
        self.declare_parameter("clip_rerank", False)
        self.declare_parameter("scene_vocabulary", [""])
        self.declare_parameter("clip_margin", 0.10)
        self.declare_parameter("clip_min_score", 0.30)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.detection_topic = str(self.get_parameter("detection_topic").value)
        self.prompt_topic = str(self.get_parameter("prompt_topic").value)
        self.remote_url = str(self.get_parameter("remote_url").value).rstrip("/")
        self.detect_url = f"{self.remote_url}/detect"
        self.request_rate_hz = max(0.1, float(self.get_parameter("request_rate_hz").value))
        self.request_timeout_sec = max(0.05, float(self.get_parameter("request_timeout_sec").value))
        self.retry_attempts = max(0, int(self.get_parameter("retry_attempts").value))
        self.max_result_staleness_sec = max(
            0.05,
            float(self.get_parameter("max_result_staleness_sec").value),
        )
        self.jpeg_quality = max(1, min(100, int(self.get_parameter("jpeg_quality").value)))
        self.max_image_long_side_px = max(0, int(self.get_parameter("max_image_long_side_px").value))
        self.default_prompt = str(self.get_parameter("default_prompt").value).strip()
        self.current_prompt = self.default_prompt
        self.box_threshold = _safe_float(self.get_parameter("box_threshold").value, 0.35)
        self.text_threshold = _safe_float(self.get_parameter("text_threshold").value, 0.25)
        self.min_score = _safe_float(self.get_parameter("min_score").value, 0.35)
        self.max_detections = max(1, int(self.get_parameter("max_detections").value))
        self.request_masks = bool(self.get_parameter("request_masks").value) and (
            np is not None and cv2 is not None)
        self.mask_topic = str(self.get_parameter("mask_topic").value)
        self.clip_rerank = bool(self.get_parameter("clip_rerank").value)
        self.scene_vocabulary = [
            str(v).strip() for v in (self.get_parameter("scene_vocabulary").value or [])
            if str(v).strip()]
        self.clip_margin = _safe_float(self.get_parameter("clip_margin").value, 0.10)
        self.clip_min_score = _safe_float(self.get_parameter("clip_min_score").value, 0.30)
        self.metrics_log_interval_sec = max(
            1.0,
            float(self.get_parameter("metrics_log_interval_sec").value),
        )

        self.bridge = CvBridge()

        self.latest_image_msg: Optional[Image] = None
        self.latest_image_receive_time = self.get_clock().now()
        self.latest_image_index = 0
        self.last_submitted_index = -1
        self.request_in_flight = False
        self.shutdown_requested = False
        self.worker_thread: Optional[threading.Thread] = None
        self.state_lock = threading.Lock()

        self.total_requests = 0
        self.total_failures = 0
        self.total_stale_results = 0
        self.total_published_messages = 0
        self.total_published_detections = 0
        self.total_skipped_in_flight = 0
        self.total_skipped_no_prompt = 0
        self.total_latency_sec = 0.0
        self.metrics_period_start = time.monotonic()

        if cv2 is None:
            self.get_logger().error("OpenCV (cv2) is not available. Remote detection client cannot run.")

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.prompt_sub = self.create_subscription(
            String,
            self.prompt_topic,
            self.prompt_callback,
            10,
        )
        self.detection_pub = self.create_publisher(Detection2DArray, self.detection_topic, 10)
        self.mask_pub = self.create_publisher(Image, self.mask_topic, 1)

        self.request_timer = self.create_timer(1.0 / self.request_rate_hz, self.request_timer_callback)
        self.metrics_timer = self.create_timer(
            self.metrics_log_interval_sec,
            self.metrics_timer_callback,
        )

        self.get_logger().info(
            "RemoteDetectionClient initialized: "
            f"image={self.image_topic} detection={self.detection_topic} "
            f"url={self.detect_url} rate={self.request_rate_hz:.2f}Hz"
        )
        if self.current_prompt:
            self.get_logger().info(f"Initial detection prompt: '{self.current_prompt}'")
        else:
            self.get_logger().warn(
                f"Detection prompt is empty. Publish to {self.prompt_topic} to start detections."
            )

    def image_callback(self, msg: Image):
        with self.state_lock:
            self.latest_image_msg = msg
            self.latest_image_receive_time = self.get_clock().now()
            self.latest_image_index += 1

    def prompt_callback(self, msg: String):
        self.current_prompt = msg.data.strip()
        if self.current_prompt:
            self.get_logger().info(f"Updated detection prompt: '{self.current_prompt}'")
        else:
            self.get_logger().warn("Detection prompt cleared. Requests will pause until prompt is set.")

    def _publish_target_mask(self, mask_png, header, width: int, height: int) -> None:
        """Decode a base64-PNG mask, resize to the source/depth resolution, publish mono8."""
        if not mask_png or np is None or cv2 is None:
            return
        try:
            raw = base64.b64decode(mask_png)
            arr = np.frombuffer(raw, dtype=np.uint8)
            mask = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)  # detector-resolution mask
            if mask is None:
                return
            if mask.shape[1] != width or mask.shape[0] != height:
                mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)
            mask = (mask > 127).astype(np.uint8) * 255
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
            mask_msg.header = header
            self.mask_pub.publish(mask_msg)
        except Exception as exc:  # pragma: no cover - mask is best-effort
            self.get_logger().warn(f"Failed to publish target mask: {exc}")

    def request_timer_callback(self):
        if cv2 is None:
            return

        with self.state_lock:
            if self.request_in_flight:
                self.total_skipped_in_flight += 1
                return
            if self.latest_image_msg is None:
                return
            if self.latest_image_index == self.last_submitted_index:
                return

            prompt = self.current_prompt.strip() if self.current_prompt else ""
            if not prompt:
                self.total_skipped_no_prompt += 1
                return

            image_msg = self.latest_image_msg
            image_receive_time = self.latest_image_receive_time
            self.last_submitted_index = self.latest_image_index
            self.request_in_flight = True
            self.total_requests += 1

        self.worker_thread = threading.Thread(
            target=self._request_worker,
            args=(image_msg, image_receive_time, prompt),
            daemon=True,
        )
        self.worker_thread.start()

    def _request_worker(self, image_msg: Image, image_receive_time, prompt: str):
        started = time.monotonic()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            source_height, source_width = cv_image.shape[:2]
            image_b64, detector_width, detector_height = _encode_image_payload(
                cv_image,
                jpeg_quality=self.jpeg_quality,
                max_long_side_px=self.max_image_long_side_px,
            )

            # Multi-label prompt: ground DINO on the whole scene vocabulary so it
            # proposes every object; CLIP (server-side) then selects the requested
            # one. Falls back to the single prompt when re-rank is off.
            dino_prompt = prompt
            clip_fields: Dict[str, Any] = {}
            if self.clip_rerank and self.scene_vocabulary:
                vocab = list(self.scene_vocabulary)
                if prompt and prompt not in vocab:
                    vocab = [prompt] + vocab
                dino_prompt = " . ".join(vocab)
                clip_fields = {
                    "clip_rerank": True,
                    "target_label": prompt,
                    "candidate_labels": vocab,
                    "clip_margin": self.clip_margin,
                    "clip_min_score": self.clip_min_score,
                }

            payload = {
                "image": image_b64,
                "image_encoding": "jpeg",
                "image_width": detector_width,
                "image_height": detector_height,
                "prompt": dino_prompt,
                **clip_fields,
                "stamp": {
                    "sec": int(image_msg.header.stamp.sec),
                    "nanosec": int(image_msg.header.stamp.nanosec),
                },
                "frame_id": image_msg.header.frame_id,
                "box_threshold": self.box_threshold,
                "text_threshold": self.text_threshold,
                "max_detections": self.max_detections,
                "return_masks": self.request_masks,
            }

            response = None
            last_error = None
            for attempt in range(self.retry_attempts + 1):
                try:
                    response = _post_json(self.detect_url, payload, self.request_timeout_sec)
                    break
                except Exception as exc:  # pragma: no cover - retry path is hard to deterministically trigger
                    last_error = exc
                    if attempt < self.retry_attempts:
                        continue

            if response is None:
                self.total_failures += 1
                self.get_logger().warn(f"Remote detection failed: {last_error}")
                return

            detections = response.get("detections", [])
            if not isinstance(detections, list):
                self.total_failures += 1
                self.get_logger().warn("Remote detector response has invalid 'detections' field.")
                return

            detections = _rescale_detections_to_source_image(
                detections=detections,
                source_width=source_width,
                source_height=source_height,
                detector_width=detector_width,
                detector_height=detector_height,
            )

            age_sec = (self.get_clock().now() - image_receive_time).nanoseconds / 1e9
            if age_sec > self.max_result_staleness_sec:
                self.total_stale_results += 1
                self.get_logger().debug(
                    "Dropping stale detection result "
                    f"(age={age_sec:.3f}s > {self.max_result_staleness_sec:.3f}s)."
                )
                return

            # CLIP says the requested object cannot be confidently picked out of the
            # candidates -> publish no detection so the grasp pipeline declines
            # rather than grabbing the wrong object.
            if response.get("ambiguous"):
                empty = Detection2DArray()
                empty.header = image_msg.header
                self.detection_pub.publish(empty)
                self.total_published_messages += 1
                top = detections[0] if detections else {}
                self.get_logger().warn(
                    f"Ambiguous match for '{prompt}' among {len(detections)} "
                    f"candidates; declining. best: P(target)={top.get('clip_target_score', 0.0):.2f} "
                    f"argmax='{top.get('clip_argmax_label', '?')}' "
                    f"P(argmax)={top.get('clip_argmax_score', 0.0):.2f} "
                    f"(need P>={self.clip_min_score:.2f}, margin>={self.clip_margin:.2f})")
                return

            detections_msg = _build_detection_message(
                header=image_msg.header,
                detections=detections,
                min_score=self.min_score,
                max_detections=self.max_detections,
            )
            self.detection_pub.publish(detections_msg)
            self.total_published_messages += 1
            self.total_published_detections += len(detections_msg.detections)
            self.total_latency_sec += max(0.0, time.monotonic() - started)

            # Publish the top detection's MobileSAM mask (aligned to the source
            # image / depth resolution) for the grasp estimate.
            if self.request_masks and detections:
                self._publish_target_mask(
                    detections[0].get("mask_png"),
                    image_msg.header,
                    source_width,
                    source_height,
                )
        except Exception as exc:
            self.total_failures += 1
            self.get_logger().warn(f"Remote detection worker error: {exc}")
        finally:
            with self.state_lock:
                self.request_in_flight = False

    def metrics_timer_callback(self):
        now = time.monotonic()
        elapsed = max(1e-6, now - self.metrics_period_start)
        req_rate = self.total_requests / elapsed
        pub_rate = self.total_published_messages / elapsed
        avg_det = (
            self.total_published_detections / self.total_published_messages
            if self.total_published_messages > 0
            else 0.0
        )
        avg_latency_ms = (
            (self.total_latency_sec / self.total_published_messages) * 1000.0
            if self.total_published_messages > 0
            else 0.0
        )

        self.get_logger().info(
            "RemoteDetection metrics: "
            f"req={req_rate:.2f}Hz pub={pub_rate:.2f}Hz avg_det={avg_det:.2f} "
            f"avg_latency={avg_latency_ms:.1f}ms failures={self.total_failures} "
            f"stale={self.total_stale_results} in_flight_skips={self.total_skipped_in_flight} "
            f"no_prompt_skips={self.total_skipped_no_prompt}"
        )

        self.total_requests = 0
        self.total_failures = 0
        self.total_stale_results = 0
        self.total_published_messages = 0
        self.total_published_detections = 0
        self.total_skipped_in_flight = 0
        self.total_skipped_no_prompt = 0
        self.total_latency_sec = 0.0
        self.metrics_period_start = now

    def destroy_node(self):
        self.shutdown_requested = True
        if hasattr(self, "request_timer") and self.request_timer is not None:
            self.request_timer.cancel()
        if hasattr(self, "metrics_timer") and self.metrics_timer is not None:
            self.metrics_timer.cancel()
        if hasattr(self, "image_sub") and self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
        if hasattr(self, "prompt_sub") and self.prompt_sub is not None:
            self.destroy_subscription(self.prompt_sub)
            self.prompt_sub = None
        if self.worker_thread is not None and self.worker_thread.is_alive():
            if threading.current_thread() is not self.worker_thread:
                self.worker_thread.join(timeout=0.5)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RemoteDetectionClientNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok():
            try:
                executor.spin_once(timeout_sec=0.1)
            except RuntimeError as exc:
                if node.shutdown_requested or not rclpy.ok():
                    break
                if "Unable to convert call argument to Python object" in str(exc):
                    if not rclpy.ok():
                        break
                raise
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
