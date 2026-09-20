"""The pointing control law.

Lives in one place so the live node and the closed-loop simulator execute the
SAME code. If they diverge, the simulator proves nothing about the system that
actually ships (CLAUDE.md rule 10).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class ControlGains:
    """Tuning for the pointing loop.

    Attributes:
        deg_per_px: Angular size of one pixel — horizontal FOV divided by frame
            width. CALIBRATE THIS. An error here shows up as sluggish or
            oscillatory tracking and is usually mistaken for a bad gain.
        proportional_gain: Loop gain. Above ~1.0 an SG90 gimbal goes
            under-damped and hunts.
        max_step_deg: Per-tick, per-axis command ceiling. The firmware also
            rate-limits; this is the host half of enforcing-twice.
        deadband_px: Below this error, do not command. SG90s have a pulse-width
            deadband of ~1-2 degrees and merely buzz when asked for less.
    """

    deg_per_px: float
    proportional_gain: float = 0.6
    max_step_deg: float = 6.0
    deadband_px: float = 3.0

    @classmethod
    def from_config(cls, config: dict, frame_width: int) -> "ControlGains":
        """Build from the `camera` and `control` blocks of bench.yaml."""
        return cls(
            deg_per_px=config["camera"]["horizontal_fov_deg"] / float(frame_width),
            proportional_gain=config["control"]["proportional_gain"],
            max_step_deg=config["control"]["max_step_deg"],
            deadband_px=config["control"]["deadband_px"],
        )


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def compute_correction(
    aim_x: float,
    aim_y: float,
    frame_center: Tuple[float, float],
    gains: ControlGains,
    feedforward_deg: Tuple[float, float] = (0.0, 0.0),
) -> Optional[Tuple[float, float]]:
    """Convert pixel error into a gimbal angle correction.

    Proportional term plus optional velocity feed-forward.

    WHY FEED-FORWARD EXISTS. A pure P controller tracking a moving target has
    irreducible steady-state error: it needs a persistent error to produce a
    persistent slew rate. With Kp=0.6, 0.0508 deg/px and a 50 Hz tick, the
    gimbal slews 1.52 deg/s per pixel of error, so holding a 20 deg/s target
    requires a standing 13.1 px error — and anything above ~22.9 deg/s pushes
    that error outside the 15 px HOLD band, meaning HOLD never latches and the
    engagement never reaches OPERATOR_AUTH.

    Lead prediction does NOT fix this. Once the loop is tracking, the target's
    apparent velocity in image space falls to zero — that is what tracking
    means — so an image-space predictor has nothing left to predict. Prediction
    helps during acquisition and manoeuvre transients, not in steady state.

    Feed-forward fixes it directly: commanding the target's own angular rate
    means zero error is a valid equilibrium.

    Args:
        aim_x: Target pixel, horizontal — normally the PREDICTED aimpoint.
        aim_y: Target pixel, vertical.
        frame_center: Boresight ``(cx, cy)``.
        gains: Tuning.
        feedforward_deg: ``(pan, tilt)`` degrees to add this tick, matching the
            target's estimated world angular rate. Defaults to no feed-forward.

    Returns:
        ``(delta_pan_deg, delta_tilt_deg)``, or None when the error is inside
        the deadband AND there is no feed-forward to apply. None is a normal
        result meaning 'already on target'.
    """
    cx, cy = frame_center
    error_x = aim_x - cx
    error_y = aim_y - cy
    ff_pan, ff_tilt = feedforward_deg

    inside_deadband = (
        abs(error_x) < gains.deadband_px and abs(error_y) < gains.deadband_px
    )
    no_feedforward = abs(ff_pan) < 1e-6 and abs(ff_tilt) < 1e-6
    if inside_deadband and no_feedforward:
        return None

    # Pan increases to the right. Tilt is inverted: image y grows downward while
    # elevation grows upward.
    delta_pan = clamp(
        error_x * gains.deg_per_px * gains.proportional_gain + ff_pan,
        -gains.max_step_deg, gains.max_step_deg,
    )
    delta_tilt = clamp(
        -error_y * gains.deg_per_px * gains.proportional_gain + ff_tilt,
        -gains.max_step_deg, gains.max_step_deg,
    )
    return (delta_pan, delta_tilt)
