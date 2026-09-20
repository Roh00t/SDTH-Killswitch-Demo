"""Camera bring-up probe — run this FIRST, before tuning anything.

Answers the three questions the latency budget depends on:
  1. Which device index is the HBVCam?
  2. Does it actually sustain 30 fps, and does MJPG change that?
  3. Is CAP_PROP_BUFFERSIZE honoured on this machine? (Expect 'no' on macOS.)

Usage:
    python -m tools.camera_probe
    python -m tools.camera_probe --index 0 --seconds 5
"""
from __future__ import annotations

import argparse
import platform
import sys
import time
from typing import List, Tuple

import cv2

TEST_MODES: List[Tuple[int, int]] = [(640, 480), (800, 600), (1280, 720), (1920, 1080)]


def backend_for_platform() -> Tuple[int, str]:
    system = platform.system()
    if system == "Darwin":
        return cv2.CAP_AVFOUNDATION, "AVFoundation"
    if system == "Linux":
        return cv2.CAP_V4L2, "V4L2"
    if system == "Windows":
        return cv2.CAP_DSHOW, "DirectShow"
    return cv2.CAP_ANY, "auto"


def enumerate_devices(max_index: int = 5) -> List[int]:
    backend, _ = backend_for_platform()
    found = []
    for index in range(max_index):
        cap = cv2.VideoCapture(index, backend)
        if cap.isOpened():
            ok, _ = cap.read()
            if ok:
                found.append(index)
        cap.release()
    return found


def measure_mode(index: int, width: int, height: int, use_mjpg: bool, seconds: float):
    """Open at a mode and measure sustained fps. Returns a result dict or None."""
    backend, _ = backend_for_platform()
    cap = cv2.VideoCapture(index, backend)
    if not cap.isOpened():
        return None

    if use_mjpg:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    buffersize_ok = bool(cap.set(cv2.CAP_PROP_BUFFERSIZE, 1))

    ok, frame = cap.read()
    if not ok or frame is None:
        cap.release()
        return None

    actual = (frame.shape[1], frame.shape[0])
    frames, latencies = 0, []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        t0 = time.monotonic()
        ok, _ = cap.read()
        if not ok:
            break
        latencies.append((time.monotonic() - t0) * 1000.0)
        frames += 1

    cap.release()
    if not latencies:
        return None

    latencies.sort()
    return {
        "requested": (width, height),
        "actual": actual,
        "mjpg": use_mjpg,
        "fps": frames / seconds,
        "read_ms_median": latencies[len(latencies) // 2],
        "read_ms_p95": latencies[int(len(latencies) * 0.95)],
        "buffersize_ok": buffersize_ok,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--index", type=int, default=None, help="Device index (default: probe)")
    parser.add_argument("--seconds", type=float, default=3.0, help="Seconds per mode")
    args = parser.parse_args()

    _, backend_name = backend_for_platform()
    print(f"Platform : {platform.system()}  |  OpenCV {cv2.__version__}  |  backend {backend_name}")

    if args.index is None:
        print("\nEnumerating devices...")
        devices = enumerate_devices()
        if not devices:
            print("  No cameras found. Check the cable and macOS camera permissions")
            print("  (System Settings > Privacy & Security > Camera > Terminal).")
            return 1
        print(f"  Working indices: {devices}")
        index = devices[0]
    else:
        index = args.index

    print(f"\nProbing index {index} ({args.seconds}s per mode)\n")
    header = f"{'mode':>11} {'fmt':>5} {'actual':>11} {'fps':>7} {'med ms':>8} {'p95 ms':>8}"
    print(header)
    print("-" * len(header))

    results = []
    for width, height in TEST_MODES:
        for use_mjpg in (True, False):
            r = measure_mode(index, width, height, use_mjpg, args.seconds)
            if r is None:
                continue
            results.append(r)
            print(
                f"{width:>5}x{height:<5} {'MJPG' if use_mjpg else 'YUYV':>5} "
                f"{r['actual'][0]:>5}x{r['actual'][1]:<5} {r['fps']:>7.1f} "
                f"{r['read_ms_median']:>8.1f} {r['read_ms_p95']:>8.1f}"
            )

    if not results:
        print("  No mode produced frames.")
        return 1

    print()
    if not results[0]["buffersize_ok"]:
        print("CAP_PROP_BUFFERSIZE : IGNORED by this backend (expected on macOS).")
        print("                      Stale-frame rejection relies on the grabber")
        print("                      thread in helper/vision/frame_source.py.")
    else:
        print("CAP_PROP_BUFFERSIZE : honoured (grabber thread still used).")

    best = max(results, key=lambda r: (round(r["fps"]), r["actual"][0] * r["actual"][1]))
    print(
        f"\nRecommended        : {best['actual'][0]}x{best['actual'][1]} "
        f"{'MJPG' if best['mjpg'] else 'YUYV'} @ {best['fps']:.1f} fps"
    )
    print(f"Capture budget     : ~{1000.0 / best['fps']:.0f} ms/frame")
    print("\nPut these numbers into architecture.md section 8, replacing the estimates.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
