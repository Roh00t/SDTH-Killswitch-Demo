"""Closed-loop simulator — proves the control law and predictor converge.

A static fixture cannot validate a feedback loop: if the detection never moves
in response to a gimbal command, nothing about the gain or the lead time is
tested. Here the synthetic target's apparent pixel position is computed from the
angular difference between the target and the gimbal, so commanding the gimbal
genuinely changes what the camera sees.

The gimbal model includes SG90 slew-rate limiting, so mechanical lag — the
dominant pole of the real loop — is in the simulation too.

Crucially this imports `compute_correction` from helper.state.control: the exact
function the live node runs. A reimplementation here would prove nothing.

Usage:
    python -m tools.simulator
    python -m tools.simulator --target-speed 25 --no-prediction
"""
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from typing import List, Optional, Tuple

from helper.state.control import ControlGains, compute_correction
from helper.vision.aimpoint import AimpointSolver
from helper.vision.predictor import AimpointPredictor
from helper.vision.types import Detection

# SG90: ~100 ms per 60 degrees -> ~600 deg/s unloaded. Firmware rate-limits to
# ~250 deg/s, which is the effective ceiling and what we model.
SERVO_SLEW_DEG_PER_S: float = 250.0


@dataclass
class SimulatedGimbal:
    """Gimbal with slew-rate limiting and no position feedback.

    `actual` is what the hardware is doing; `commanded` is what the host asked
    for. The host can only observe `commanded` — exactly like the real SG90 —
    so any convergence shown here is achieved without cheating on feedback.
    """

    actual_pan: float = 90.0
    actual_tilt: float = 90.0
    commanded_pan: float = 90.0
    commanded_tilt: float = 90.0
    pan_min: float = 0.0
    pan_max: float = 180.0
    tilt_min: float = 45.0
    tilt_max: float = 135.0

    def command(self, pan: float, tilt: float) -> None:
        """Absolute command, clamped as the driver and firmware both clamp."""
        self.commanded_pan = max(self.pan_min, min(self.pan_max, pan))
        self.commanded_tilt = max(self.tilt_min, min(self.tilt_max, tilt))

    def step(self, dt: float) -> None:
        """Advance mechanics toward the commanded position."""
        max_travel = SERVO_SLEW_DEG_PER_S * dt
        for axis in ("pan", "tilt"):
            actual = getattr(self, f"actual_{axis}")
            target = getattr(self, f"commanded_{axis}")
            delta = target - actual
            if abs(delta) > max_travel:
                delta = math.copysign(max_travel, delta)
            setattr(self, f"actual_{axis}", actual + delta)


@dataclass
class SimulatedTarget:
    """A target at a world bearing, optionally moving.

    Supports constant-rate travel and sinusoidal weave. The weave matters: a
    constant-rate target eventually leaves the gimbal arc, whereas a real drone
    manoeuvres within a volume, which is the harder tracking problem anyway
    because it inverts the velocity the predictor is leading on.
    """

    azimuth_deg: float = 100.0
    elevation_deg: float = 95.0
    az_rate_deg_s: float = 0.0
    el_rate_deg_s: float = 0.0
    az_weave_deg: float = 0.0
    az_weave_hz: float = 0.0
    apparent_width_px: float = 120.0
    _elapsed: float = 0.0
    _base_az: Optional[float] = None

    def step(self, dt: float) -> None:
        if self._base_az is None:
            self._base_az = self.azimuth_deg
        self._elapsed += dt
        self._base_az += self.az_rate_deg_s * dt
        self.elevation_deg += self.el_rate_deg_s * dt
        weave = (
            self.az_weave_deg * math.sin(2.0 * math.pi * self.az_weave_hz * self._elapsed)
            if self.az_weave_hz > 0.0 else 0.0
        )
        self.azimuth_deg = self._base_az + weave


class SimulatedWorld:
    """Projects a target's world bearing into camera pixels.

    Pixel position is a function of (target bearing - gimbal ACTUAL bearing), so
    the loop is genuinely closed: commanding the gimbal moves the target in
    frame, which changes the next correction.
    """

    def __init__(
        self,
        frame_width: int = 1280,
        frame_height: int = 720,
        horizontal_fov_deg: float = 65.0,
    ) -> None:
        self.frame_size = (frame_width, frame_height)
        self.center = (frame_width / 2.0, frame_height / 2.0)
        self.px_per_deg = frame_width / horizontal_fov_deg

    def observe(
        self, target: SimulatedTarget, gimbal: SimulatedGimbal, track_id: int = 1
    ) -> Optional[Detection]:
        """Render the target, or None if it is outside the frame."""
        d_az = target.azimuth_deg - gimbal.actual_pan
        d_el = target.elevation_deg - gimbal.actual_tilt

        px = self.center[0] + d_az * self.px_per_deg
        py = self.center[1] - d_el * self.px_per_deg  # image y grows downward

        if not (0 <= px < self.frame_size[0] and 0 <= py < self.frame_size[1]):
            return None

        return Detection(
            x=px, y=py,
            w=target.apparent_width_px, h=target.apparent_width_px,
            confidence=0.92, class_id=0, class_name="drone", track_id=track_id,
        )


@dataclass
class RunResult:
    """Outcome of one simulated engagement."""

    converged: bool
    ticks_to_converge: Optional[int]
    final_error_px: float
    steady_state_error_px: float
    peak_error_px: float
    lost_target: bool
    errors: List[float]

    def __str__(self) -> str:
        status = "CONVERGED" if self.converged else "DID NOT CONVERGE"
        ticks = self.ticks_to_converge if self.ticks_to_converge is not None else "-"
        return (
            f"{status:<18} ticks={ticks:<5} "
            f"final={self.final_error_px:6.2f}px  "
            f"steady={self.steady_state_error_px:6.2f}px  "
            f"peak={self.peak_error_px:7.2f}px"
            + ("  [TARGET LOST]" if self.lost_target else "")
        )


def run_engagement(
    target: SimulatedTarget,
    gains: ControlGains,
    *,
    use_prediction: bool = True,
    lead_time_s: float = 0.09,
    tick_hz: float = 50.0,
    detection_hz: Optional[float] = None,
    ticks: int = 400,
    hold_threshold_px: float = 15.0,
    world: Optional[SimulatedWorld] = None,
) -> RunResult:
    """Run a closed-loop engagement and report convergence.

    Args:
        target: The thing being tracked.
        gains: Control tuning — the same object the node builds from config.
        use_prediction: When False, aim at the last observation instead of the
            predicted position. This is the A/B that shows whether the velocity
            estimator earns its place.
        lead_time_s: Lead applied when predicting. In the node this comes from
            LatencyTracker.total_lead_s, measured live.
        tick_hz: Control loop rate.
        detection_hz: Rate at which NEW detections arrive. None means every
            tick. Set this to the model's real inference rate to simulate the
            decoupling that actually exists: the tick loop runs at 100 Hz but
            a 216 ms model only delivers ~4.6 observations per second, so the
            controller spends most ticks extrapolating from a stale snapshot.
        ticks: Maximum iterations.
        hold_threshold_px: Error band counted as converged.

    Returns:
        A RunResult. `steady_state_error_px` is the mean over the final quarter
        of the run, which is the number that matters for HOLD.
    """
    world = world or SimulatedWorld(horizontal_fov_deg=gains.deg_per_px * 1280.0)
    gimbal = SimulatedGimbal()
    solver = AimpointSolver()
    predictor = AimpointPredictor(smoothing=0.4)

    dt = 1.0 / tick_hz
    px_per_deg = 1.0 / gains.deg_per_px
    errors: List[float] = []
    converged_at: Optional[int] = None
    lost = False
    elapsed = 0.0
    gimbal_rate = (0.0, 0.0)  # deg/s, from the previous tick's command
    detection_period = (1.0 / detection_hz) if detection_hz else 0.0
    last_detection_at = -1e9
    held_detection = None

    for tick in range(ticks):
        # Observations arrive only as fast as the model can produce them.
        if detection_period <= 0.0 or (elapsed - last_detection_at) >= detection_period:
            held_detection = world.observe(target, gimbal)
            last_detection_at = elapsed
            fresh = True
        else:
            fresh = False
        detection = held_detection

        if detection is None:
            lost = True
            errors.append(float("inf"))
            target.step(dt)
            gimbal.step(dt)
            elapsed += dt
            continue

        aim = solver.solve(detection)
        if fresh:
            # Only feed the estimator genuinely new observations; re-feeding a
            # stale one would read as zero velocity and kill the feed-forward.
            predictor.update(aim, timestamp=elapsed)

        feedforward = (0.0, 0.0)
        if use_prediction:
            predicted = predictor.predict(lead_time_s)
            aim_x, aim_y = predicted if predicted is not None else (aim.x, aim.y)

            # Target world rate = apparent image rate + the gimbal's own rate.
            # Without the second term the estimate collapses to zero exactly
            # when the loop starts working.
            velocity = predictor.velocity_px_s
            if velocity is not None:
                # Feed-forward must advance by one OBSERVATION interval, not one
                # tick. At 4.6 fps those differ by 20x, and using the tick
                # interval under-feeds the loop by the same factor.
                obs_dt = detection_period if detection_period > 0 else dt
                world_pan_rate = velocity[0] / px_per_deg + gimbal_rate[0]
                world_tilt_rate = -velocity[1] / px_per_deg + gimbal_rate[1]
                feedforward = (world_pan_rate * obs_dt, world_tilt_rate * obs_dt)
        else:
            aim_x, aim_y = aim.x, aim.y

        error = math.hypot(aim.x - world.center[0], aim.y - world.center[1])
        errors.append(error)
        if converged_at is None and error <= hold_threshold_px:
            converged_at = tick

        # Command only on a fresh observation — mirrors the node's frame_id
        # gate. Stepping a feedback loop on stale feedback winds it up.
        if fresh:
            correction = compute_correction(
                aim_x, aim_y, world.center, gains, feedforward_deg=feedforward
            )
            if correction is not None:
                gimbal.command(
                    gimbal.commanded_pan + correction[0],
                    gimbal.commanded_tilt + correction[1],
                )
                obs_dt = detection_period if detection_period > 0 else dt
                gimbal_rate = (correction[0] / obs_dt, correction[1] / obs_dt)
            else:
                gimbal_rate = (0.0, 0.0)

        target.step(dt)
        gimbal.step(dt)
        elapsed += dt

    tail = [e for e in errors[-(ticks // 4):] if math.isfinite(e)]
    finite = [e for e in errors if math.isfinite(e)]
    return RunResult(
        converged=converged_at is not None,
        ticks_to_converge=converged_at,
        final_error_px=errors[-1] if math.isfinite(errors[-1]) else float("inf"),
        steady_state_error_px=sum(tail) / len(tail) if tail else float("inf"),
        peak_error_px=max(finite) if finite else float("inf"),
        lost_target=lost,
        errors=errors,
    )


def default_gains(fov_deg: float = 65.0, width: int = 1280, kp: float = 0.6) -> ControlGains:
    return ControlGains(
        deg_per_px=fov_deg / width, proportional_gain=kp,
        max_step_deg=6.0, deadband_px=3.0,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target-speed", type=float, default=20.0,
                        help="Target azimuth rate, deg/s")
    parser.add_argument("--kp", type=float, default=0.6, help="Proportional gain")
    parser.add_argument("--lead-ms", type=float, default=90.0, help="Lead time, ms")
    parser.add_argument("--ticks", type=int, default=400)
    args = parser.parse_args()

    gains = default_gains(kp=args.kp)
    lead = args.lead_ms / 1000.0

    print(f"\nClosed-loop convergence  (Kp={args.kp}, lead={args.lead_ms:.0f}ms, "
          f"{gains.deg_per_px:.4f} deg/px)")
    print("=" * 78)

    print("\n1. Static target, 10 deg off boresight")
    print("   ", run_engagement(SimulatedTarget(azimuth_deg=100.0, elevation_deg=90.0),
                                gains, lead_time_s=lead, ticks=args.ticks))

    print("\n2. Static target, diagonal offset")
    print("   ", run_engagement(SimulatedTarget(azimuth_deg=105.0, elevation_deg=100.0),
                                gains, lead_time_s=lead, ticks=args.ticks))

    def ab_test(label: str, make_target, ticks: int) -> None:
        with_pred = run_engagement(make_target(), gains, use_prediction=True,
                                   lead_time_s=lead, ticks=ticks)
        without_pred = run_engagement(make_target(), gains, use_prediction=False,
                                      lead_time_s=lead, ticks=ticks)
        print(f"\n{label}")
        print("    with prediction   :", with_pred)
        print("    without prediction:", without_pred)
        a, b = with_pred.steady_state_error_px, without_pred.steady_state_error_px
        if math.isfinite(a) and math.isfinite(b) and b > 0:
            print(f"    -> prediction reduces steady-state error "
                  f"{b:.2f}px -> {a:.2f}px ({(1 - a / b) * 100:.1f}%)")

    # 150 ticks at 50 Hz = 3 s; at 20 deg/s that is 60 deg of travel, which
    # stays inside the 180 deg pan arc.
    ab_test(f"3. Crossing target at {args.target_speed:.0f} deg/s — prediction A/B",
            lambda: SimulatedTarget(azimuth_deg=100.0, elevation_deg=90.0,
                                    az_rate_deg_s=args.target_speed), 150)

    ab_test("3b. Weaving target, +/-15 deg at 0.4 Hz — prediction A/B",
            lambda: SimulatedTarget(azimuth_deg=95.0, elevation_deg=90.0,
                                    az_weave_deg=15.0, az_weave_hz=0.4), args.ticks)

    print("\n4. Gain sweep — static target")
    for kp in (0.2, 0.4, 0.6, 0.8, 1.2, 1.8):
        result = run_engagement(
            SimulatedTarget(azimuth_deg=100.0, elevation_deg=90.0),
            default_gains(kp=kp), lead_time_s=lead, ticks=args.ticks)
        print(f"    Kp={kp:<4} {result}")

    print("\nSteady-state error is the number that matters: HOLD requires it to stay")
    print("inside +/-15px for 3.0 continuous seconds.\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
