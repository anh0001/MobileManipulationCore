#!/usr/bin/env python3
"""Test script for OpenVLA policy server."""

import base64
import json
import sys
import time
import requests
from pathlib import Path


def test_inference(image_path: str, task: str = "pick up the cardboard box"):
    """Send an image and task to the OpenVLA server and print the response."""

    # Read and encode the image
    with open(image_path, "rb") as f:
        image_bytes = f.read()
    image_b64 = base64.b64encode(image_bytes).decode("utf-8")

    # Prepare the request
    request_data = {
        "image": image_b64,
        "task": task,
        "reference_frame": "base_link"
    }

    # Send POST request to the inference endpoint
    url = "http://100.112.100.20:30542/infer"
    print(f"Sending inference request to {url}")
    print(f"Task: {task}")
    print(f"Image: {image_path}")
    print("-" * 80)

    start_time = time.time()
    response = requests.post(url, json=request_data, timeout=60)
    inference_time = time.time() - start_time

    if response.status_code == 200:
        result = response.json()
        print("SUCCESS! Policy output received:")
        print(json.dumps(result, indent=2))

        # Print human-readable summary
        print("\n" + "=" * 80)
        print("SUMMARY (OpenVLA outputs DELTA actions - incremental changes):")
        print("=" * 80)
        if result.get("has_eef_target"):
            pos = result["eef_target_pose"]["position"]
            orient = result["eef_target_pose"]["orientation"]
            print(f"End-effector DELTA position: dx={pos['x']:.4f}, dy={pos['y']:.4f}, dz={pos['z']:.4f} (meters)")
            print(f"End-effector DELTA orientation: x={orient['x']:.4f}, y={orient['y']:.4f}, z={orient['z']:.4f}, w={orient['w']:.4f} (quaternion)")

        if result.get("gripper_active"):
            gripper_cmd = result.get("gripper_command", 0.0)
            gripper_state = "OPEN" if gripper_cmd > 0.5 else "CLOSED"
            print(f"Gripper command: {gripper_cmd:.4f} ({gripper_state})")

        print(f"Confidence: {result.get('confidence', 0.0):.4f}")
        print(f"Reference frame: {result.get('reference_frame', 'N/A')}")
        print(f"Inference time: {inference_time:.3f} seconds ({1.0/inference_time:.2f} Hz)")
    else:
        print(f"ERROR! Status code: {response.status_code}")
        print(f"Response: {response.text}")
        sys.exit(1)


if __name__ == "__main__":
    # Default to test image in data directory
    default_image = Path(__file__).parent / "data" / "test_image01.png"
    image_path = sys.argv[1] if len(sys.argv) > 1 else str(default_image)
    task = sys.argv[2] if len(sys.argv) > 2 else "pick up the cardboard box"

    test_inference(image_path, task)
