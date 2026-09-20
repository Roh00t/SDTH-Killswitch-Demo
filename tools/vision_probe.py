"""Live detection diagnostic — shows what the model ACTUALLY sees.

When the node sits in SCAN and never acquires, the question is always the same:
is the camera dead, is the model blind, or is the filter throwing away good
detections? This answers it directly by showing every class the model emits at
a deliberately low confidence floor, with per-class counts and peak confidence.

Run this pointed at whatever you are trying to detect. If boxes appear here but
the node does not acquire, the problem is the threshold or the class filter,
not the model.

Controls:  Q quit  ·  UP/DOWN adjust confidence floor  ·  S save frame

Usage:
    python -m tools.vision_probe
    python -m tools.vision_probe --conf 0.02 --device 0
"""
from __future__ import annotations

import argparse
import sys
import time
from collections import defaultdict
from typing import Dict

import cv2
import numpy as np

from helper.vision.detector import UltralyticsDetector
from helper.vision.frame_source import UsbCameraSource

_PALETTE = {
    "drone": (80, 220, 80), "bird": (60, 180, 250),
    "airplane": (250, 180, 60), "helicopter": (200, 120, 240),
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/bench.yaml")
    parser.add_argument("--conf", type=float, default=0.05,
                        help="Confidence floor. Low on purpose — we want to see "
                             "what the model is thinking, not what it is sure of.")
    parser.add_argument("--device", type=int, default=None)
    args = parser.parse_args()

    import yaml
    with open(args.config, "r", encoding="utf-8") as handle:
        cfg = yaml.safe_load(handle)

    cam_cfg, det_cfg = cfg["camera"], cfg["detector"]
    conf_floor = args.conf

    print(f"Loading {det_cfg['weights']} ...")
    # NO class filter: we want to see every class, including the confusers.
    detector = UltralyticsDetector(
        det_cfg["weights"], conf_threshold=conf_floor,
        target_classes=None, imgsz=det_cfg["imgsz"],
    )
    node_conf = det_cfg["conf_threshold"]
    node_classes = set(det_cfg["target_classes"])
    print(f"Node would use: conf>={node_conf}  classes={sorted(node_classes)}")
    print(f"Probe is using: conf>={conf_floor}  classes=ALL\n")

    camera = UsbCameraSource(
        device_index=args.device if args.device is not None else cam_cfg["device_index"],
        width=cam_cfg["width"], height=cam_cfg["height"],
        target_fps=cam_cfg["target_fps"], use_mjpg=cam_cfg["use_mjpg"],
    )
    camera.start()

    peak: Dict[str, float] = defaultdict(float)
    counts: Dict[str, int] = defaultdict(int)
    would_pass = 0
    frames = 0
    last_id = -1
    fps = 0.0
    started = time.monotonic()
    window = "Killswitch // Vision Probe"
    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            frame, frame_id = camera.read()
            if frame is None or frame_id == last_id:
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue
            last_id = frame_id

            t0 = time.monotonic()
            detections = detector.detect(frame)
            infer_ms = (time.monotonic() - t0) * 1000.0
            frames += 1
            fps = frames / max(1e-6, time.monotonic() - started)

            canvas = frame.copy()
            for d in detections:
                counts[d.class_name] += 1
                peak[d.class_name] = max(peak[d.class_name], d.confidence)
                passes = d.confidence >= node_conf and d.class_name in node_classes
                if passes:
                    would_pass += 1
                colour = _PALETTE.get(d.class_name, (200, 200, 200))
                x1, y1, x2, y2 = (int(v) for v in d.xyxy)
                cv2.rectangle(canvas, (x1, y1), (x2, y2), colour, 2 if passes else 1)
                tag = f"{d.class_name} {d.confidence:.2f}{' PASS' if passes else ''}"
                cv2.putText(canvas, tag, (x1, max(16, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2 if passes else 1)

            h = canvas.shape[0]
            cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 30), (20, 20, 24), -1)
            cv2.putText(canvas,
                        f"floor={conf_floor:.2f}  node_conf={node_conf:.2f}  "
                        f"{infer_ms:.0f}ms  {fps:.1f}fps  raw={len(detections)}  "
                        f"would_pass={would_pass}",
                        (10, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (240, 240, 240), 1)

            y = h - 10 - 20 * len(counts)
            for name in sorted(counts):
                cv2.putText(canvas,
                            f"{name:<11} seen {counts[name]:<5} peak {peak[name]:.2f}",
                            (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            _PALETTE.get(name, (200, 200, 200)), 1)
                y += 20

            cv2.imshow(window, canvas)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                path = f"captures/probe-{int(time.time())}.jpg"
                import os
                os.makedirs("captures", exist_ok=True)
                cv2.imwrite(path, canvas)
                print(f"saved {path}")
            elif key == 82:      # up
                conf_floor = min(0.95, conf_floor + 0.05)
                detector._conf = conf_floor
            elif key == 84:      # down
                conf_floor = max(0.01, conf_floor - 0.05)
                detector._conf = conf_floor
    finally:
        camera.stop()
        cv2.destroyAllWindows()

    print("\n--- summary ---")
    print(f"frames processed : {frames}")
    if not counts:
        print("NOTHING DETECTED AT ALL, in any class, at any confidence.")
        print("  -> the model is not responding to this scene. Check that the")
        print("     target fills a reasonable part of the frame, and that the")
        print("     training data resembles what you are showing it.")
    else:
        for name in sorted(counts):
            print(f"  {name:<12} detections {counts[name]:<6} peak conf {peak[name]:.3f}")
        print(f"\nwould have passed the node's filter "
              f"(conf>={node_conf}, {sorted(node_classes)}): {would_pass}")
        if would_pass == 0:
            print("  -> the model SEES things but nothing survives the node filter.")
            print(f"     Lower detector.conf_threshold below the peak above, or widen")
            print(f"     detector.target_classes in config/bench.yaml.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
