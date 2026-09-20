"""Lead prediction and runtime latency measurement.

Aiming where the target was seen guarantees a miss by exactly the loop latency
multiplied by the target's angular rate. This module measures that latency
continuously and leads the aimpoint by it.

Why not ByteTrack's internal Kalman: it is reachable only via private
attributes (`predictor.trackers[0].tracked_stracks[i].mean`) whose shape shifts
between ultralytics releases. An explicit estimator here is version-independent
and unit-testable.
"""
from __future__ import annotations

import logging
import time
from typing import Optional, Tuple

from helper.vision.types import AimPoint

logger = logging.getLogger(__name__)


class LatencyTracker:
    """Rolling estimate of true glass-to-command latency.

    Fed with (frame capture time, command issue time) pairs from the live loop,
    so the lead time reflects what the system is actually doing rather than a
    figure typed into a config file.

    Thread-safety: not thread-safe. Owned by the state machine thread.
    """

    def __init__(
        self,
        initial_estimate_s: float = 0.060,
        mechanical_allowance_s: float = 0.060,
        alpha: float = 0.2,
    ) -> None:
        """Args:
            initial_estimate_s: Seed for the compute-side estimate, used until
                real samples arrive.
            mechanical_allowance_s: Fixed allowance for servo travel. Cannot be
                measured: SG90s have no position feedback, so this is an
                estimate and is labelled as one everywhere it surfaces.
            alpha: EWMA weight on each new sample.
        """
        if not 0.0 < alpha <= 1.0:
            raise ValueError(f"alpha must be in (0, 1], got {alpha}")
        self._compute_s = initial_estimate_s
        self._mechanical_s = mechanical_allowance_s
        self._alpha = alpha
        self._samples = 0

    def record(self, captured_at: float, commanded_at: float) -> None:
        """Record one measured capture-to-command interval."""
        measured = commanded_at - captured_at
        if measured <= 0.0 or measured > 1.0:
            # A negative or absurd sample means a clock or threading fault;
            # discard rather than poisoning the estimate.
            logger.debug("Discarding implausible latency sample %.3fs", measured)
            return
        self._compute_s += self._alpha * (measured - self._compute_s)
        self._samples += 1

    @property
    def compute_latency_s(self) -> float:
        """Measured glass-to-serial-write latency. The figure software owns."""
        return self._compute_s

    @property
    def total_lead_s(self) -> float:
        """Compute latency plus mechanical allowance — the true lead time."""
        return self._compute_s + self._mechanical_s

    @property
    def sample_count(self) -> int:
        return self._samples

    def summary(self) -> str:
        return (
            f"compute {self._compute_s * 1000:.0f}ms (measured, n={self._samples}) "
            f"+ mechanical {self._mechanical_s * 1000:.0f}ms (estimated) "
            f"= {self.total_lead_s * 1000:.0f}ms lead"
        )


class AimpointPredictor:
    """Estimates aimpoint velocity and projects it forward by the lead time.

    Operates on the AIMPOINT, not the box centre: if a weak-point offset is
    applied, that is the point we are driving to and therefore the point whose
    motion matters.

    Thread-safety: not thread-safe. Owned by the state machine thread.
    """

    def __init__(
        self,
        smoothing: float = 0.4,
        max_lead_px: float = 250.0,
        max_sample_gap_s: float = 0.5,
    ) -> None:
        """Args:
            smoothing: EWMA weight on each new velocity sample. Higher is more
                responsive and noisier.
            max_lead_px: Cap on applied lead. An unbounded lead from a noisy
                velocity estimate would slew the gimbal off the target.
            max_sample_gap_s: Samples further apart than this restart the
                estimate — the track was effectively lost in between.
        """
        if not 0.0 < smoothing <= 1.0:
            raise ValueError(f"smoothing must be in (0, 1], got {smoothing}")
        self._smoothing = smoothing
        self._max_lead_px = max_lead_px
        self._max_gap_s = max_sample_gap_s

        self._last_xy: Optional[Tuple[float, float]] = None
        self._last_t: Optional[float] = None
        self._velocity: Optional[Tuple[float, float]] = None
        self._track_id: Optional[int] = None

    def reset(self) -> None:
        """Drop all history. Called on track change and on entry to IDLE."""
        self._last_xy = None
        self._last_t = None
        self._velocity = None
        self._track_id = None

    def update(self, aim: AimPoint, timestamp: Optional[float] = None) -> None:
        """Feed a new aimpoint observation.

        A change of track_id resets the estimate: a new identity is a different
        object, and carrying velocity across would fling the gimbal.
        """
        now = timestamp if timestamp is not None else time.monotonic()

        if aim.track_id != self._track_id:
            self.reset()
            self._track_id = aim.track_id

        if self._last_xy is not None and self._last_t is not None:
            dt = now - self._last_t
            if dt <= 0.0 or dt > self._max_gap_s:
                self._velocity = None
            else:
                vx = (aim.x - self._last_xy[0]) / dt
                vy = (aim.y - self._last_xy[1]) / dt
                if self._velocity is None:
                    self._velocity = (vx, vy)
                else:
                    a = self._smoothing
                    self._velocity = (
                        self._velocity[0] + a * (vx - self._velocity[0]),
                        self._velocity[1] + a * (vy - self._velocity[1]),
                    )

        self._last_xy = (aim.x, aim.y)
        self._last_t = now

    def predict(self, lead_time_s: float) -> Optional[Tuple[float, float]]:
        """Project the aimpoint forward.

        Args:
            lead_time_s: Seconds to lead by — normally LatencyTracker.total_lead_s.

        Returns:
            Predicted ``(x, y)``, or None if velocity is not yet established.
            None means 'aim at the last observation', which is the correct
            degraded behaviour, not an error.
        """
        if self._last_xy is None or self._velocity is None:
            return None

        lead_x = self._velocity[0] * lead_time_s
        lead_y = self._velocity[1] * lead_time_s

        magnitude = (lead_x * lead_x + lead_y * lead_y) ** 0.5
        if magnitude > self._max_lead_px:
            scale = self._max_lead_px / magnitude
            lead_x *= scale
            lead_y *= scale

        return (self._last_xy[0] + lead_x, self._last_xy[1] + lead_y)

    @property
    def velocity_px_s(self) -> Optional[Tuple[float, float]]:
        """Current smoothed velocity estimate, px/s, or None."""
        return self._velocity

    @property
    def speed_px_s(self) -> float:
        """Scalar speed, 0.0 when no estimate exists."""
        if self._velocity is None:
            return 0.0
        return (self._velocity[0] ** 2 + self._velocity[1] ** 2) ** 0.5
