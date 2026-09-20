"""Model sanity check against still images — no camera involved.

Answers the one question that matters when live detection fails: is the model
broken, or is the live scene simply nothing like its training data?

Run it on images from your Colab dataset. A model that scores well on its own
validation images but fails on your webcam has a DOMAIN GAP — the fix is a
better test target or more varied training data. A model that fails on its own
validation images is UNDERTRAINED — the fix is retraining, and no amount of
threshold tuning will rescue it.

Usage:
    python -m tools.model_sanity path/to/image.jpg
    python -m tools.model_sanity path/to/val_images/
    python -m tools.model_sanity imgs/ --conf 0.05 --save-annotated
"""
from __future__ import annotations

import argparse
import sys
from collections import defaultdict
from pathlib import Path
from typing import List

import cv2

from helper.vision.detector import UltralyticsDetector

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp", ".tif", ".tiff"}


def collect_images(target: Path) -> List[Path]:
    if target.is_file():
        return [target]
    if target.is_dir():
        return sorted(
            p for p in target.rglob("*") if p.suffix.lower() in IMAGE_SUFFIXES
        )
    return []


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", help="Image file or directory")
    parser.add_argument("--config", default="config/bench.yaml")
    parser.add_argument("--conf", type=float, default=0.05,
                        help="Confidence floor for the probe (default 0.05)")
    parser.add_argument("--save-annotated", action="store_true",
                        help="Write annotated copies to captures/sanity/")
    parser.add_argument("--limit", type=int, default=40)
    args = parser.parse_args()

    import yaml
    with open(args.config, "r", encoding="utf-8") as handle:
        cfg = yaml.safe_load(handle)
    det_cfg = cfg["detector"]

    images = collect_images(Path(args.path))
    if not images:
        print(f"No images found at {args.path}")
        return 1
    images = images[: args.limit]

    print(f"Loading {det_cfg['weights']} ...")
    detector = UltralyticsDetector(
        det_cfg["weights"], conf_threshold=args.conf,
        target_classes=None, imgsz=det_cfg["imgsz"],
    )
    node_conf = det_cfg["conf_threshold"]
    node_classes = set(det_cfg["target_classes"])

    print(f"\nProbing {len(images)} image(s) at conf>={args.conf}, all classes.")
    print(f"Node filter for comparison: conf>={node_conf}, {sorted(node_classes)}\n")
    print(f"{'image':<34}{'raw':>5}{'pass':>6}  best detection")
    print("-" * 78)

    out_dir = Path("captures/sanity")
    if args.save_annotated:
        out_dir.mkdir(parents=True, exist_ok=True)

    totals = defaultdict(int)
    peaks = defaultdict(float)
    images_with_pass = 0
    images_with_any = 0

    for path in images:
        frame = cv2.imread(str(path))
        if frame is None:
            print(f"{path.name[:33]:<34}  unreadable")
            continue

        detector.reset()
        detections = detector.detect(frame)
        passing = [
            d for d in detections
            if d.confidence >= node_conf and d.class_name in node_classes
        ]
        if detections:
            images_with_any += 1
        if passing:
            images_with_pass += 1

        for d in detections:
            totals[d.class_name] += 1
            peaks[d.class_name] = max(peaks[d.class_name], d.confidence)

        best = max(detections, key=lambda d: d.confidence, default=None)
        summary = (
            f"{best.class_name} {best.confidence:.3f} "
            f"({best.w:.0f}x{best.h:.0f}px)" if best else "-- nothing --"
        )
        print(f"{path.name[:33]:<34}{len(detections):>5}{len(passing):>6}  {summary}")

        if args.save_annotated and detections:
            for d in detections:
                x1, y1, x2, y2 = (int(v) for v in d.xyxy)
                ok = d.confidence >= node_conf and d.class_name in node_classes
                colour = (80, 220, 80) if ok else (160, 160, 160)
                cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
                cv2.putText(frame, f"{d.class_name} {d.confidence:.2f}",
                            (x1, max(16, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, colour, 2)
            cv2.imwrite(str(out_dir / path.name), frame)

    print("\n" + "=" * 78)
    print(f"images with ANY detection      : {images_with_any}/{len(images)}")
    print(f"images passing the node filter : {images_with_pass}/{len(images)}")
    if totals:
        print("\nper-class totals:")
        for name in sorted(totals):
            print(f"  {name:<12} {totals[name]:>5} detections   peak conf {peaks[name]:.3f}")

    print("\nVERDICT")
    if images_with_any == 0:
        print("  The model detects NOTHING in these images, at any confidence.")
        print("  If these are its own training/validation images, the model is")
        print("  broken or was exported wrong. Retraining is the only fix.")
    elif images_with_pass == 0:
        print("  The model sees things but nothing clears the node's filter.")
        print(f"  Peak confidence across all classes: {max(peaks.values()):.3f}")
        print(f"  Node requires {node_conf}. Either the model is undertrained, or")
        print("  the class filter excludes what it is actually finding.")
    elif images_with_pass < len(images) * 0.5:
        print(f"  Only {images_with_pass}/{len(images)} images produce a usable")
        print("  detection. That is too unreliable for a live engagement demo.")
    else:
        print(f"  {images_with_pass}/{len(images)} images detect cleanly. The model")
        print("  works on THIS kind of imagery. If live camera detection still")
        print("  fails, the problem is a domain gap: lighting, scale, background")
        print("  or sensor differences between these images and your scene.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
