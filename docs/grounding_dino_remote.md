# Grounding DINO Remote Detection Guide

This guide describes the remote detection path for visual-servo mode:

- Jetson node: `remote_detection_client` (ROS 2 topic bridge)
- Remote server: `detection_server` (HTTP Grounding DINO inference)
- Output topic to visual servo: `/manipulation/target_detections` (`vision_msgs/msg/Detection2DArray`)

## Architecture

1. Jetson subscribes to camera image and compresses it as JPEG.
2. Jetson sends `POST /detect` to remote server with prompt + image payload.
3. Remote server runs Grounding DINO and returns normalized detections.
4. Jetson republishes detections to `/manipulation/target_detections`.
5. `visual_servo_node` uses detections for acquire/re-seed while local tracker runs at control rate.

## Request/Response Contract

### `POST /detect` Request (Jetson -> server)

- `image` (string): base64 JPEG bytes
- `image_encoding` (string): `jpeg`
- `image_width` (int): encoded image width
- `image_height` (int): encoded image height
- `prompt` (string): runtime prompt from `/visual_servo/target_prompt`
- `stamp.sec` (int, optional): source image timestamp seconds
- `stamp.nanosec` (int, optional): source image timestamp nanoseconds
- `frame_id` (string, optional): source frame id
- `box_threshold` (float, optional)
- `text_threshold` (float, optional)
- `max_detections` (int, optional)

### `POST /detect` Response (server -> Jetson)

- `detections` (array):
  - `class_id` (string)
  - `score` (float)
  - `cx`, `cy` (float): bbox center in pixels
  - `w`, `h` (float): bbox size in pixels
- `inference_ms` (float, optional)

## Jetson Configuration

Edit `config/detection_params.yaml`:

- `remote_url`: detector server URL
- `request_rate_hz`: request frequency (recommended `4.0`)
- `request_timeout_sec`: request timeout (recommended `0.30`)
- `max_result_staleness_sec`: stale result drop window (recommended `0.40`)
- `jpeg_quality`: JPEG quality (recommended `70`)
- `max_image_long_side_px`: max encoded image long side (recommended `640`)
- `default_prompt`: startup prompt when no prompt topic message was sent yet
- `prompt_topic`: runtime prompt topic (`/visual_servo/target_prompt`)

## Remote Server Setup

On the remote GPU machine (with `manipulation_policy` installed):

```bash
python3 -m manipulation_policy.detection_server --host 0.0.0.0 --port 30543
```

Optional environment variables:

- `GROUNDING_DINO_MODEL_ID` (default: `IDEA-Research/grounding-dino-base`)
- `GROUNDING_DINO_DEVICE` (e.g., `cuda`)
- `GROUNDING_DINO_DEFAULT_PROMPT`
- `GROUNDING_DINO_BOX_THRESHOLD` (default `0.35`)
- `GROUNDING_DINO_TEXT_THRESHOLD` (default `0.25`)
- `GROUNDING_DINO_MAX_DETECTIONS` (default `5`)

Health check:

```bash
curl http://<SERVER_IP>:30543/health
```

## Runtime Commands

On Jetson:

```bash
# Launch visual-servo mode (starts remote_detection_client automatically)
ros2 launch manipulation_bringup core_launch.py control_mode:=visual_servo

# Publish runtime prompt updates
ros2 run manipulation_policy detection_prompt_cli
```

Monitor:

```bash
ros2 topic echo /manipulation/target_detections
ros2 topic hz /manipulation/target_detections
ros2 topic echo /visual_servo/state
```
