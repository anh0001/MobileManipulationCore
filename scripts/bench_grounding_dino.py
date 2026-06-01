#!/usr/bin/env python3
"""Benchmark Grounding DINO inference latency on this device (Jetson)."""
import os
import time

import torch
from PIL import Image
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection

IMG = os.getenv("BENCH_IMAGE", "/workspace/img.png")
PROMPT = os.getenv("BENCH_PROMPT", "a bottle.")
LONG_SIDE = int(os.getenv("BENCH_LONG_SIDE", "640"))
ITERS = int(os.getenv("BENCH_ITERS", "20"))
MODELS = os.getenv("BENCH_MODELS", "IDEA-Research/grounding-dino-tiny,IDEA-Research/grounding-dino-base").split(",")

device = "cuda" if torch.cuda.is_available() else "cpu"
print(f"device={device} torch={torch.__version__} cuda={torch.version.cuda} "
      f"name={torch.cuda.get_device_name(0) if device=='cuda' else 'cpu'}")

img = Image.open(IMG).convert("RGB")
w, h = img.size
scale = LONG_SIDE / max(w, h)
if scale < 1.0:
    img = img.resize((int(w * scale), int(h * scale)))
print(f"image={IMG} size={img.size} prompt={PROMPT!r} iters={ITERS}")


FP16 = os.getenv("BENCH_FP16", "0") == "1"


def bench(model_id):
    t0 = time.time()
    proc = AutoProcessor.from_pretrained(model_id)
    model = AutoModelForZeroShotObjectDetection.from_pretrained(model_id).to(device).eval()
    load_s = time.time() - t0
    use_amp = FP16 and device == "cuda"

    def once():
        inputs = proc(images=img, text=PROMPT, return_tensors="pt").to(device)
        with torch.no_grad():
            if use_amp:
                with torch.autocast(device_type="cuda", dtype=torch.float16):
                    model(**inputs)
            else:
                model(**inputs)
        if device == "cuda":
            torch.cuda.synchronize()

    # warmup
    for _ in range(3):
        once()
    times = []
    for _ in range(ITERS):
        s = time.time()
        once()
        times.append((time.time() - s) * 1000.0)
    times.sort()
    mean = sum(times) / len(times)
    p50 = times[len(times) // 2]
    p90 = times[int(len(times) * 0.9)]
    print(f"\n== {model_id} ==")
    print(f"  load:   {load_s:6.1f} s")
    print(f"  mean:   {mean:6.1f} ms  ({1000.0/mean:4.1f} FPS)")
    print(f"  p50:    {p50:6.1f} ms   p90: {p90:6.1f} ms")
    print(f"  min:    {min(times):6.1f} ms  max: {max(times):6.1f} ms")
    del model
    if device == "cuda":
        torch.cuda.empty_cache()


for m in MODELS:
    try:
        bench(m.strip())
    except Exception as e:  # noqa: BLE001
        print(f"\n== {m} == FAILED: {e}")
