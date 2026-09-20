"""Annotated video link to the operator console.

The node owns the camera, so the console cannot open it — two processes cannot
share a USB device. The node therefore annotates frames and publishes them, and
the console displays them. That is also how a real C2 console works: it watches
a remote node rather than sitting on its sensor.

This is a best-effort link. Publishing must never block or fail the control
loop; a dropped frame is a cosmetic problem, a stalled tick loop is not.
"""
from __future__ import annotations

import logging
import time
from typing import Optional, Tuple

import cv2
import numpy as np

from helper.state.machine import TrackSnapshot

logger = logging.getLogger(__name__)

TOPIC_NODE_VIDEO: str = "c2/node/video"

# BGR. Deliberately few colours: state is read from the banner, not from hue.
_WHITE = (255, 255, 255)
_GREEN = (80, 220, 80)
_AMBER = (60, 180, 250)
_RED = (60, 60, 240)
_GREY = (150, 150, 150)

_STATE_COLOUR = {
    "IDLE": _GREY, "SCAN": _AMBER, "TRACK": _GREEN,
    "HOLD": _GREEN, "OPERATOR_AUTH": _AMBER, "ENGAGE": _RED,
}


def annotate(
    frame: np.ndarray,
    snapshot: Optional[TrackSnapshot],
    state: str,
    hold_progress: float = 0.0,
) -> np.ndarray:
    """Draw detections, aimpoint and boresight onto a copy of the frame.

    Args:
        frame: Source BGR frame. Not mutated.
        snapshot: Latest vision result, or None.
        state: Current EngagementState value.
        hold_progress: 0.0-1.0 fraction of the HOLD duration achieved.

    Returns:
        An annotated copy.
    """
    canvas = frame.copy()
    height, width = canvas.shape[:2]
    cx, cy = width // 2, height // 2
    colour = _STATE_COLOUR.get(state, _WHITE)

    # Boresight reticle.
    cv2.line(canvas, (cx - 20, cy), (cx - 6, cy), _WHITE, 1)
    cv2.line(canvas, (cx + 6, cy), (cx + 20, cy), _WHITE, 1)
    cv2.line(canvas, (cx, cy - 20), (cx, cy - 6), _WHITE, 1)
    cv2.line(canvas, (cx, cy + 6), (cx, cy + 20), _WHITE, 1)

    if snapshot is not None:
        for detection in snapshot.detections:
            x1, y1, x2, y2 = (int(v) for v in detection.xyxy)
            is_target = snapshot.target is not None and detection is snapshot.target
            cv2.rectangle(canvas, (x1, y1), (x2, y2), colour if is_target else _GREY,
                          2 if is_target else 1)
            label = f"{detection.class_name} {detection.confidence:.2f}"
            if detection.track_id is not None:
                label += f" #{detection.track_id}"
            cv2.putText(canvas, label, (x1, max(14, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour if is_target else _GREY, 1)

        if snapshot.aim is not None:
            ax, ay = int(snapshot.aim.x), int(snapshot.aim.y)
            cv2.drawMarker(canvas, (ax, ay), colour, cv2.MARKER_CROSS, 22, 2)
            cv2.line(canvas, (cx, cy), (ax, ay), colour, 1)
            if snapshot.aim.downgraded:
                # The operator must know when weak-point biasing was gated off.
                cv2.putText(canvas, "AIMPOINT: CENTRE-OF-MASS (gated)", (ax + 14, ay),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, _AMBER, 1)

    # HOLD progress bar — the operator's cue that authorisation is imminent.
    if hold_progress > 0.0:
        bar_w = int(width * 0.4)
        x0, y0 = (width - bar_w) // 2, height - 28
        cv2.rectangle(canvas, (x0, y0), (x0 + bar_w, y0 + 10), _GREY, 1)
        cv2.rectangle(canvas, (x0, y0),
                      (x0 + int(bar_w * min(1.0, hold_progress)), y0 + 10), colour, -1)

    return canvas


def encode_jpeg(frame: np.ndarray, max_width: int = 640, quality: int = 60) -> Optional[bytes]:
    """Downscale and JPEG-encode for the MQTT link.

    640px at q60 is roughly 20-30 KB, so 15 fps costs ~450 KB/s — safe on a LAN
    and far below Mosquitto's message ceiling.

    Returns:
        Encoded bytes, or None if encoding failed.
    """
    height, width = frame.shape[:2]
    if width > max_width:
        scale = max_width / float(width)
        frame = cv2.resize(frame, (max_width, int(height * scale)))
    ok, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    return buffer.tobytes() if ok else None


def decode_jpeg(payload: bytes) -> Optional[np.ndarray]:
    """Decode a received frame, or None if the payload is not a valid image."""
    array = np.frombuffer(payload, np.uint8)
    frame = cv2.imdecode(array, cv2.IMREAD_COLOR)
    return frame if frame is not None else None


class VideoPublisher:
    """Rate-limited annotated-frame publisher.

    Thread-safety: call from the main tick thread only.
    """

    def __init__(self, client, fps: float = 15.0, max_width: int = 640, quality: int = 60):
        self._client = client
        self._period = 1.0 / fps if fps > 0 else 0.0
        self._max_width = max_width
        self._quality = quality
        self._last_sent = 0.0

    def maybe_publish(
        self, frame: np.ndarray, snapshot: Optional[TrackSnapshot],
        state: str, hold_progress: float = 0.0,
    ) -> None:
        """Publish if the rate limit allows. Never raises."""
        now = time.monotonic()
        if now - self._last_sent < self._period:
            return
        self._last_sent = now
        try:
            payload = encode_jpeg(
                annotate(frame, snapshot, state, hold_progress),
                self._max_width, self._quality,
            )
            if payload is not None:
                self._client.publish_raw(TOPIC_NODE_VIDEO, payload)
        except Exception as exc:  # noqa: BLE001 - cosmetic link, never fatal
            logger.debug("Video publish skipped: %s", exc)
