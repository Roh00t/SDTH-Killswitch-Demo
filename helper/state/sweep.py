"""Search sweep and cue geometry.

Contains the corrected sweep logic. The legacy `scan_handle_x` called
`turn_servo_x(+rate)` in BOTH bounds-recovery branches (main_helper.py:51 and
:59), so hitting the upper bound commanded further travel into it. A stalled
SG90 draws locked-rotor current and cooks.

Correction: check whether the NEXT step would exceed a bound BEFORE taking it.
Never step out and correct.
"""
from __future__ import annotations

import logging
from typing import Optional, Tuple

from helper.hardware.protocol import (
    PAN_MAX_DEG,
    PAN_MIN_DEG,
    TILT_MAX_DEG,
    TILT_MIN_DEG,
    clamp_tilt,
)

logger = logging.getLogger(__name__)


class SweepController:
    """Boustrophedon search: sweep pan, step tilt at each bound, reverse.

    Thread-safety: not thread-safe. Owned by the state machine thread.
    """

    def __init__(
        self,
        pan_step_deg: float = 4.0,
        tilt_step_deg: float = 8.0,
        max_cycles: int = 2,
        pan_min: float = PAN_MIN_DEG,
        pan_max: float = PAN_MAX_DEG,
        tilt_min: float = TILT_MIN_DEG,
        tilt_max: float = TILT_MAX_DEG,
    ) -> None:
        """Args:
            pan_step_deg: Degrees per pan step.
            tilt_step_deg: Degrees of tilt per pan traverse.
            max_cycles: Complete tilt traverses before declaring the search
                exhausted. Bounded search is mandatory — no unbounded loop
                exists anywhere in the system (guardrails section 3, HARD).
        """
        if pan_step_deg <= 0 or tilt_step_deg <= 0:
            raise ValueError("sweep steps must be positive")

        self._pan_step = pan_step_deg
        self._tilt_step = tilt_step_deg
        self._max_cycles = max_cycles
        self._bounds = (pan_min, pan_max, tilt_min, tilt_max)

        self._pan = (pan_min + pan_max) / 2.0
        self._tilt = (tilt_min + tilt_max) / 2.0
        self._pan_forward = True
        self._tilt_forward = True
        self._cycles = 0

    def reset(self, pan: float, tilt: float) -> None:
        """Re-seed at the cued position and clear the cycle counter."""
        pan_min, pan_max, tilt_min, tilt_max = self._bounds
        self._pan = max(pan_min, min(pan_max, pan))
        self._tilt = max(tilt_min, min(tilt_max, tilt))
        self._pan_forward = True
        self._tilt_forward = True
        self._cycles = 0

    @property
    def position(self) -> Tuple[float, float]:
        return (self._pan, self._tilt)

    @property
    def cycles_completed(self) -> int:
        return self._cycles

    @property
    def exhausted(self) -> bool:
        """True once the search has covered its volume `max_cycles` times."""
        return self._cycles >= self._max_cycles

    def step(self) -> Tuple[float, float]:
        """Advance one step and return the new commanded position.

        Returns:
            ``(pan, tilt)`` in degrees, always inside bounds.
        """
        pan_min, pan_max, tilt_min, tilt_max = self._bounds

        direction = 1.0 if self._pan_forward else -1.0
        candidate = self._pan + direction * self._pan_step

        # THE FIX: test the candidate before committing to it. The legacy code
        # committed the step and then tried to correct, pushing into the bound.
        if pan_min <= candidate <= pan_max:
            self._pan = candidate
            return (self._pan, self._tilt)

        # At a pan bound: reverse, and step tilt one row. Pan does not move
        # this tick — it is already at the edge of its travel.
        self._pan_forward = not self._pan_forward
        self._pan = pan_max if candidate > pan_max else pan_min

        tilt_direction = 1.0 if self._tilt_forward else -1.0
        tilt_candidate = self._tilt + tilt_direction * self._tilt_step

        if tilt_min <= tilt_candidate <= tilt_max:
            self._tilt = tilt_candidate
        else:
            # Tilt bound too: one full traverse of the search volume is done.
            self._tilt_forward = not self._tilt_forward
            self._tilt = clamp_tilt(tilt_candidate)
            self._cycles += 1
            logger.debug("Sweep cycle %d/%d complete", self._cycles, self._max_cycles)

        return (self._pan, self._tilt)


def cue_to_gimbal(
    azimuth_deg: float,
    elevation_deg: float,
    boresight_az_deg: float = 0.0,
    pan_min: float = PAN_MIN_DEG,
    pan_max: float = PAN_MAX_DEG,
) -> Optional[Tuple[float, float]]:
    """Convert a C2 cue to gimbal angles, or None if unreachable.

    A 180-degree pan gimbal cannot cover 360 degrees of azimuth. Cues outside
    the reachable arc are REJECTED rather than clamped: silently slewing to the
    edge of travel and searching the wrong sky is worse than declining the cue,
    because it looks like the system is working.

    Elevation is clamped rather than rejected — the gimbal can still search a
    useful band, and an out-of-band elevation cue is a coverage limit, not a
    pointing error.

    Args:
        azimuth_deg: Cue azimuth, 0-360, compass convention.
        elevation_deg: Cue elevation, -90 to 90.
        boresight_az_deg: Azimuth the gimbal points at pan = centre.
        pan_min: Minimum pan travel.
        pan_max: Maximum pan travel.

    Returns:
        ``(pan_deg, tilt_deg)``, or None if the azimuth is outside the arc.
    """
    pan_center = (pan_min + pan_max) / 2.0
    half_arc = (pan_max - pan_min) / 2.0

    # Wrap into -180..180 relative to boresight.
    relative = ((azimuth_deg - boresight_az_deg + 180.0) % 360.0) - 180.0

    if abs(relative) > half_arc:
        logger.warning(
            "Cue az=%.1f is %.1f deg off boresight, outside the +/-%.1f gimbal "
            "arc. Cue rejected.", azimuth_deg, relative, half_arc,
        )
        return None

    return (pan_center + relative, clamp_tilt(90.0 + elevation_deg))
