"""Shared vision vocabulary.

Pure-Python by design: no numpy, no OpenCV. Targeting math must be testable
without hardware or heavyweight dependencies installed.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class Detection:
    """A single detected object in image space.

    Replaces the legacy ``(x, y)`` centre tuple, which discarded ``w``/``h`` at
    the point of computation and made aimpoint offsetting arithmetically
    impossible. All coordinates are pixels in the source frame.

    Attributes:
        x: Bounding-box centre, horizontal.
        y: Bounding-box centre, vertical.
        w: Bounding-box width.
        h: Bounding-box height.
        confidence: Detector confidence, 0.0-1.0.
        class_id: Integer class index from the model.
        class_name: Human-readable class label.
        track_id: Stable identity from the tracker, or None if untracked.
    """

    x: float
    y: float
    w: float
    h: float
    confidence: float
    class_id: int
    class_name: str
    track_id: Optional[int] = None

    @property
    def center(self) -> Tuple[float, float]:
        """Centre of mass — the legacy aimpoint, now one option among several."""
        return (self.x, self.y)

    @property
    def xyxy(self) -> Tuple[float, float, float, float]:
        """Corner format ``(x1, y1, x2, y2)``.

        Note: the legacy code fed centre-format boxes to ``cv2.dnn.NMSBoxes``,
        which expects this corner format. Use this property at any API boundary
        that wants corners.
        """
        half_w, half_h = self.w / 2.0, self.h / 2.0
        return (self.x - half_w, self.y - half_h, self.x + half_w, self.y + half_h)

    @property
    def min_dimension(self) -> float:
        """Smaller box dimension — drives the aimpoint resolution gate."""
        return min(self.w, self.h)

    @property
    def area(self) -> float:
        return self.w * self.h


@dataclass(frozen=True)
class AimPoint:
    """Where the effector should be pointed, and why.

    Carries its own provenance so the state machine and the audit log can record
    whether a weak-point offset was actually applied or silently downgraded.

    Attributes:
        x: Target pixel, horizontal.
        y: Target pixel, vertical.
        track_id: Identity this solution is bound to. Authorisation is bound to
            this value; a solution for track 7 must never fire on track 9.
        offset_applied: The ``(offset_x, offset_y)`` actually used after clamping
            and gating. ``(0.0, 0.0)`` means centre-of-mass.
        downgraded: True when the resolution gate forced centre-of-mass.
        reason: Human-readable explanation, for logs and operator display.
    """

    x: float
    y: float
    track_id: Optional[int]
    offset_applied: Tuple[float, float]
    downgraded: bool
    reason: str
