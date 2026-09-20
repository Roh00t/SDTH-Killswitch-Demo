"""Closed-loop convergence tests.

These exercise `compute_correction` — the same function main.py runs — against
a simulated gimbal with slew-rate limiting and no position feedback. A static
fixture cannot validate a feedback loop; these can.
"""
from __future__ import annotations

import math

import pytest

from helper.state.control import ControlGains, compute_correction
from tools.simulator import (
    SimulatedGimbal,
    SimulatedTarget,
    SimulatedWorld,
    default_gains,
    run_engagement,
)

HOLD_PX = 15.0


class TestStaticConvergence:
    def test_offset_target_converges_inside_hold_band(self):
        result = run_engagement(
            SimulatedTarget(azimuth_deg=100.0, elevation_deg=90.0),
            default_gains(), ticks=300,
        )
        assert result.converged
        assert result.steady_state_error_px < HOLD_PX

    def test_diagonal_offset_converges(self):
        result = run_engagement(
            SimulatedTarget(azimuth_deg=105.0, elevation_deg=100.0),
            default_gains(), ticks=300,
        )
        assert result.converged and result.steady_state_error_px < HOLD_PX

    def test_target_is_not_lost_during_a_static_engagement(self):
        result = run_engagement(
            SimulatedTarget(azimuth_deg=98.0, elevation_deg=92.0),
            default_gains(), ticks=300,
        )
        assert not result.lost_target


class TestMovingTargetAndFeedForward:
    def test_crossing_target_converges_with_prediction(self):
        result = run_engagement(
            SimulatedTarget(azimuth_deg=100.0, elevation_deg=90.0, az_rate_deg_s=20.0),
            default_gains(), use_prediction=True, ticks=150,
        )
        assert result.converged and result.steady_state_error_px < HOLD_PX

    def test_feedforward_beats_pure_proportional_on_a_ramp(self):
        """The headline result: P alone leaves a standing tracking error."""
        target = lambda: SimulatedTarget(  # noqa: E731
            azimuth_deg=100.0, elevation_deg=90.0, az_rate_deg_s=20.0
        )
        with_ff = run_engagement(target(), default_gains(), use_prediction=True, ticks=150)
        without_ff = run_engagement(target(), default_gains(), use_prediction=False, ticks=150)
        assert with_ff.steady_state_error_px < without_ff.steady_state_error_px * 0.5

    def test_weaving_target_stays_inside_hold_band_only_with_feedforward(self):
        """Without feed-forward this target sits OUTSIDE the band, so HOLD
        never latches and the engagement never reaches OPERATOR_AUTH."""
        target = lambda: SimulatedTarget(  # noqa: E731
            azimuth_deg=95.0, elevation_deg=90.0, az_weave_deg=15.0, az_weave_hz=0.4
        )
        with_ff = run_engagement(target(), default_gains(), use_prediction=True, ticks=400)
        without_ff = run_engagement(target(), default_gains(), use_prediction=False, ticks=400)
        assert with_ff.steady_state_error_px < HOLD_PX
        assert without_ff.steady_state_error_px > HOLD_PX


class TestGainStability:
    @pytest.mark.parametrize("kp", [0.2, 0.4, 0.6])
    def test_configured_gain_range_is_stable(self, kp):
        result = run_engagement(
            SimulatedTarget(azimuth_deg=100.0, elevation_deg=90.0),
            default_gains(kp=kp), ticks=300,
        )
        assert result.converged and result.steady_state_error_px < HOLD_PX

    def test_high_gain_is_under_damped(self):
        """Documents why bench.yaml ships Kp=0.6 and not something larger."""
        result = run_engagement(
            SimulatedTarget(azimuth_deg=100.0, elevation_deg=90.0),
            default_gains(kp=1.8), ticks=300,
        )
        assert result.steady_state_error_px > HOLD_PX


class TestSimulationFidelity:
    def test_gimbal_slew_is_rate_limited(self):
        gimbal = SimulatedGimbal()
        gimbal.command(180.0, 135.0)
        gimbal.step(0.02)
        assert gimbal.actual_pan < 100.0  # cannot teleport

    def test_gimbal_clamps_to_bounds(self):
        gimbal = SimulatedGimbal()
        gimbal.command(999.0, -999.0)
        assert gimbal.commanded_pan == 180.0 and gimbal.commanded_tilt == 45.0

    def test_target_outside_frame_is_not_observed(self):
        world = SimulatedWorld()
        gimbal = SimulatedGimbal(actual_pan=90.0, actual_tilt=90.0)
        assert world.observe(SimulatedTarget(azimuth_deg=175.0, elevation_deg=90.0), gimbal) is None

    def test_boresight_target_renders_at_frame_centre(self):
        world = SimulatedWorld()
        gimbal = SimulatedGimbal(actual_pan=90.0, actual_tilt=90.0)
        detection = world.observe(SimulatedTarget(azimuth_deg=90.0, elevation_deg=90.0), gimbal)
        assert detection is not None
        assert detection.x == pytest.approx(world.center[0])
        assert detection.y == pytest.approx(world.center[1])


class TestFeedForwardUnit:
    def test_deadband_still_returns_none_without_feedforward(self):
        gains = default_gains()
        assert compute_correction(640.0, 360.0, (640.0, 360.0), gains) is None

    def test_feedforward_overrides_the_deadband(self):
        """Inside the deadband but tracking a mover: still command."""
        gains = default_gains()
        correction = compute_correction(
            640.0, 360.0, (640.0, 360.0), gains, feedforward_deg=(0.4, 0.0)
        )
        assert correction is not None and correction[0] == pytest.approx(0.4)

    def test_feedforward_is_clamped_with_the_proportional_term(self):
        gains = default_gains()
        correction = compute_correction(
            1280.0, 360.0, (640.0, 360.0), gains, feedforward_deg=(50.0, 0.0)
        )
        assert correction is not None
        assert abs(correction[0]) <= gains.max_step_deg
