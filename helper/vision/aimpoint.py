"""Aimpoint solving — bounding box plus normalised offset to a target pixel.

This is the 'weak-point targeting' interface. Be precise about what it is:
a *geometric* bias within a detected box. It does not identify a rotor hub.
A trained keypoint model behind the same interface would; that is future work
and must be described as such. See architecture.md section 9.3.
"""
from __future__ import annotations

import logging
from typing import Tuple

from helper.vision.types import AimPoint, Detection

logger = logging.getLogger(__name__)

# Offsets beyond +/-0.5 would place the aimpoint outside the detected box.
MAX_ABS_OFFSET: float = 0.5

# Below this box dimension (pixels) an offset is smaller than typical
# frame-to-frame box jitter, so applying it aims at noise rather than at a
# sub-component. Rationale: a 0.35 offset on a 40 px box displaces 14 px, well
# clear of the 2-5 px jitter typical of a stable track. On a 10 px box the same
# offset displaces 3.5 px, which is inside the noise floor.
DEFAULT_MIN_BOX_PX: float = 40.0


class AimpointSolver:
    """Converts a Detection into an AimPoint by applying a normalised offset.

    The offset is expressed as a fraction of box width and height, so it scales
    automatically with range: a target that doubles in apparent size gets an
    offset that doubles in pixels, holding the same physical aimpoint.

    Thread-safety: stateless after construction. Safe to call from any thread.
    """

    def __init__(
        self,
        offset_x: float = 0.0,
        offset_y: float = 0.0,
        min_box_px: float = DEFAULT_MIN_BOX_PX,
    ) -> None:
        """Initialise the solver.

        Args:
            offset_x: Horizontal offset as a fraction of box width. Negative is
                left of centre. Clamped to +/-0.5.
            offset_y: Vertical offset as a fraction of box height. Negative is
                above centre. Clamped to +/-0.5.
            min_box_px: Minimum box dimension below which the solver downgrades
                to centre-of-mass.

        Raises:
            ValueError: If min_box_px is not positive.
        """
        if min_box_px <= 0:
            raise ValueError(f"min_box_px must be positive, got {min_box_px}")

        self._offset_x = self._clamp_offset(offset_x, "offset_x")
        self._offset_y = self._clamp_offset(offset_y, "offset_y")
        self._min_box_px = min_box_px

    @staticmethod
    def _clamp_offset(value: float, name: str) -> float:
        """Clamp an offset to +/-MAX_ABS_OFFSET, logging if it was out of range."""
        clamped = max(-MAX_ABS_OFFSET, min(MAX_ABS_OFFSET, value))
        if clamped != value:
            logger.warning(
                "%s=%.3f exceeds +/-%.1f and was clamped to %.3f; an aimpoint "
                "may not fall outside its detected box",
                name, value, MAX_ABS_OFFSET, clamped,
            )
        return clamped

    @property
    def configured_offset(self) -> Tuple[float, float]:
        """The post-clamp offset this solver will apply when not gated."""
        return (self._offset_x, self._offset_y)

    def solve(self, detection: Detection) -> AimPoint:
        """Compute the aimpoint for a detection.

        Applies the configured offset unless the resolution gate trips, in which
        case it returns centre-of-mass and marks the result as downgraded.

        Args:
            detection: The target.

        Returns:
            An AimPoint carrying the pixel target and its provenance.
        """
        if detection.min_dimension < self._min_box_px:
            return AimPoint(
                x=detection.x,
                y=detection.y,
                track_id=detection.track_id,
                offset_applied=(0.0, 0.0),
                downgraded=True,
                reason=(
                    f"resolution gate: box min dimension "
                    f"{detection.min_dimension:.1f}px < {self._min_box_px:.1f}px "
                    f"threshold; offset would be inside jitter, using centre-of-mass"
                ),
            )

        aim_x = detection.x + (self._offset_x * detection.w)
        aim_y = detection.y + (self._offset_y * detection.h)

        if self._offset_x == 0.0 and self._offset_y == 0.0:
            reason = "centre-of-mass (no offset configured)"
        else:
            reason = (
                f"offset ({self._offset_x:+.2f}, {self._offset_y:+.2f}) applied to "
                f"{detection.w:.0f}x{detection.h:.0f}px box"
            )

        return AimPoint(
            x=aim_x,
            y=aim_y,
            track_id=detection.track_id,
            offset_applied=(self._offset_x, self._offset_y),
            downgraded=False,
            reason=reason,
        )


def select_priority_target(
    detections, frame_center: Tuple[float, float]
):
    """Pick the detection nearest frame centre.

    Replaces the legacy ``get_closest_coords``, which operated on bare centre
    tuples and returned a sentinel ``(9999, 9999)`` when its hardcoded distance
    ceiling was never beaten. This returns None instead of a sentinel, so a
    caller cannot mistake 'no target' for a target at the far corner.

    Args:
        detections: Candidate detections.
        frame_center: ``(cx, cy)`` of the frame.

    Returns:
        The nearest Detection, or None if the sequence is empty.
    """
    if not detections:
        return None

    cx, cy = frame_center
    return min(detections, key=lambda d: (d.x - cx) ** 2 + (d.y - cy) ** 2)


def pixel_error(aim: AimPoint, frame_center: Tuple[float, float]) -> Tuple[float, float]:
    """Signed pixel error from frame centre to the aimpoint.

    Positive x means the aimpoint is right of centre; positive y means below.
    The control law consumes this directly; the HOLD gate consumes its magnitude.

    Args:
        aim: The solved aimpoint.
        frame_center: ``(cx, cy)`` of the frame.

    Returns:
        ``(error_x, error_y)`` in pixels.
    """
    cx, cy = frame_center
    return (aim.x - cx, aim.y - cy)


def error_magnitude(aim: AimPoint, frame_center: Tuple[float, float]) -> float:
    """Euclidean pixel error from frame centre — the HOLD gate quantity."""
    ex, ey = pixel_error(aim, frame_center)
    return (ex * ex + ey * ey) ** 0.5
