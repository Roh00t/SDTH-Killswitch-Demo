"""State machine, sweep and prediction tests. No hardware, no model, no broker."""
from __future__ import annotations

import pytest

from helper.state.machine import (
    LEGAL_TRANSITIONS,
    EngagementState,
    IllegalTransitionError,
    SnapshotHolder,
    StateMachine,
    TrackSnapshot,
)
from helper.state.sweep import SweepController, cue_to_gimbal
from helper.vision.aimpoint import AimpointSolver
from helper.vision.predictor import AimpointPredictor, LatencyTracker
from helper.vision.types import Detection

S = EngagementState


def det(x=320.0, y=240.0, w=100.0, h=100.0, track_id=1) -> Detection:
    return Detection(x=x, y=y, w=w, h=h, confidence=0.9,
                     class_id=0, class_name="drone", track_id=track_id)


def advance(machine: StateMachine, *states: EngagementState) -> None:
    for state in states:
        machine.transition_to(state, "test")


class TestTransitionTable:
    def test_engage_has_exactly_one_predecessor(self):
        """The core safety invariant of the whole system."""
        predecessors = [s for s, allowed in LEGAL_TRANSITIONS.items() if S.ENGAGE in allowed]
        assert predecessors == [S.OPERATOR_AUTH]

    def test_idle_is_reachable_from_every_state(self):
        for state, allowed in LEGAL_TRANSITIONS.items():
            if state is not S.IDLE:
                assert S.IDLE in allowed, f"{state} cannot reach IDLE"

    def test_every_state_has_an_entry(self):
        assert set(LEGAL_TRANSITIONS) == set(S)


class TestIllegalTransitions:
    @pytest.mark.parametrize("target", [S.TRACK, S.HOLD, S.OPERATOR_AUTH, S.ENGAGE])
    def test_idle_cannot_skip_ahead(self, target):
        with pytest.raises(IllegalTransitionError):
            StateMachine().transition_to(target, "test")

    def test_cannot_reach_engage_without_authorisation(self):
        machine = StateMachine()
        advance(machine, S.SCAN, S.TRACK, S.HOLD)
        with pytest.raises(IllegalTransitionError, match="ENGAGE"):
            machine.transition_to(S.ENGAGE, "skipping the operator")

    def test_scan_cannot_jump_to_hold(self):
        machine = StateMachine()
        advance(machine, S.SCAN)
        with pytest.raises(IllegalTransitionError):
            machine.transition_to(S.HOLD, "test")

    def test_engage_can_only_go_to_idle(self):
        machine = StateMachine()
        advance(machine, S.SCAN, S.TRACK, S.HOLD, S.OPERATOR_AUTH, S.ENGAGE)
        for target in (S.SCAN, S.TRACK, S.HOLD, S.OPERATOR_AUTH):
            with pytest.raises(IllegalTransitionError):
                machine.transition_to(target, "test")
        machine.transition_to(S.IDLE, "burn complete")
        assert machine.state is S.IDLE


class TestHappyPath:
    def test_full_engagement_sequence(self):
        machine = StateMachine()
        advance(machine, S.SCAN, S.TRACK, S.HOLD, S.OPERATOR_AUTH, S.ENGAGE, S.IDLE)
        assert machine.state is S.IDLE
        assert [t.to_state for t in machine.history()] == [
            S.SCAN, S.TRACK, S.HOLD, S.OPERATOR_AUTH, S.ENGAGE, S.IDLE,
        ]

    def test_re_entry_is_a_noop_not_an_error(self):
        machine = StateMachine()
        machine.transition_to(S.IDLE, "already here")
        assert machine.state is S.IDLE
        assert machine.history() == ()


class TestForceIdle:
    @pytest.mark.parametrize(
        "path",
        [(S.SCAN,), (S.SCAN, S.TRACK), (S.SCAN, S.TRACK, S.HOLD),
         (S.SCAN, S.TRACK, S.HOLD, S.OPERATOR_AUTH),
         (S.SCAN, S.TRACK, S.HOLD, S.OPERATOR_AUTH, S.ENGAGE)],
    )
    def test_force_idle_works_from_everywhere(self, path):
        """Fault handlers must never be blocked by the transition table."""
        machine = StateMachine()
        advance(machine, *path)
        machine.force_idle("fault")
        assert machine.state is S.IDLE
        assert machine.history()[-1].reason.startswith("FORCED")

    def test_transition_callback_fires(self):
        seen = []
        machine = StateMachine(on_transition=seen.append)
        machine.transition_to(S.SCAN, "cue")
        machine.force_idle("fault")
        assert [t.to_state for t in seen] == [S.SCAN, S.IDLE]


class TestSweepBoundsBug:
    """The legacy sweep commanded further travel INTO a bound it had hit."""

    def test_pan_never_exceeds_bounds_over_a_long_run(self):
        sweep = SweepController(pan_step_deg=7.0, tilt_step_deg=11.0, max_cycles=99)
        sweep.reset(90.0, 90.0)
        for _ in range(2000):
            pan, tilt = sweep.step()
            assert 0.0 <= pan <= 180.0, f"pan escaped to {pan}"
            assert 45.0 <= tilt <= 135.0, f"tilt escaped to {tilt}"

    def test_reversal_moves_away_from_the_bound(self):
        sweep = SweepController(pan_step_deg=10.0, tilt_step_deg=10.0)
        sweep.reset(175.0, 90.0)
        first, _ = sweep.step()        # would be 185 -> clamps to the bound
        assert first == 180.0
        second, _ = sweep.step()       # must now move AWAY, not further out
        assert second < first

    def test_tilt_advances_when_pan_reverses(self):
        sweep = SweepController(pan_step_deg=10.0, tilt_step_deg=10.0)
        sweep.reset(175.0, 90.0)
        _, tilt_before = sweep.position
        _, tilt_after = sweep.step()
        assert tilt_after != tilt_before

    def test_search_is_bounded_and_terminates(self):
        """No unbounded search loop exists anywhere in the system."""
        sweep = SweepController(pan_step_deg=15.0, tilt_step_deg=20.0, max_cycles=2)
        sweep.reset(90.0, 90.0)
        for _ in range(5000):
            if sweep.exhausted:
                break
            sweep.step()
        assert sweep.exhausted

    def test_rejects_non_positive_steps(self):
        with pytest.raises(ValueError):
            SweepController(pan_step_deg=0.0)


class TestCueGeometry:
    def test_boresight_cue_maps_to_pan_centre(self):
        assert cue_to_gimbal(0.0, 0.0, boresight_az_deg=0.0) == (90.0, 90.0)

    def test_cue_outside_the_arc_is_rejected_not_clamped(self):
        """Slewing to the edge and searching the wrong sky looks like working."""
        assert cue_to_gimbal(180.0, 0.0, boresight_az_deg=0.0) is None

    def test_wraps_across_north(self):
        result = cue_to_gimbal(350.0, 0.0, boresight_az_deg=10.0)
        assert result is not None and result[0] == pytest.approx(70.0)

    def test_elevation_is_clamped_to_gimbal_travel(self):
        result = cue_to_gimbal(0.0, 80.0, boresight_az_deg=0.0)
        assert result is not None and result[1] == 135.0


class TestPrediction:
    def test_constant_velocity_leads_correctly(self):
        predictor = AimpointPredictor(smoothing=1.0)
        solver = AimpointSolver()
        for i in range(5):  # 100 px/s rightward at 10 Hz
            predictor.update(solver.solve(det(x=100.0 + i * 10.0)), timestamp=i * 0.1)
        assert predictor.speed_px_s == pytest.approx(100.0, rel=0.05)
        predicted = predictor.predict(0.1)
        assert predicted is not None and predicted[0] == pytest.approx(150.0, rel=0.05)

    def test_no_prediction_before_velocity_is_established(self):
        """Returns None -> caller aims at last observation. Degraded, not broken."""
        predictor = AimpointPredictor()
        predictor.update(AimpointSolver().solve(det()), timestamp=0.0)
        assert predictor.predict(0.1) is None

    def test_identity_switch_resets_velocity(self):
        predictor = AimpointPredictor(smoothing=1.0)
        solver = AimpointSolver()
        for i in range(4):
            predictor.update(solver.solve(det(x=100.0 + i * 20.0, track_id=1)), timestamp=i * 0.1)
        assert predictor.speed_px_s > 0.0
        predictor.update(solver.solve(det(x=500.0, track_id=2)), timestamp=0.4)
        assert predictor.predict(0.1) is None

    def test_lead_is_capped(self):
        predictor = AimpointPredictor(smoothing=1.0, max_lead_px=50.0)
        solver = AimpointSolver()
        for i in range(4):  # 10000 px/s — noise, not a real target
            predictor.update(solver.solve(det(x=i * 1000.0)), timestamp=i * 0.1)
        predicted = predictor.predict(1.0)
        assert predicted is not None
        assert abs(predicted[0] - 3000.0) <= 50.0 + 1e-6


class TestLatencyTracker:
    def test_converges_on_measured_latency(self):
        tracker = LatencyTracker(initial_estimate_s=0.200, mechanical_allowance_s=0.05, alpha=0.5)
        for i in range(30):
            tracker.record(captured_at=i * 1.0, commanded_at=i * 1.0 + 0.040)
        assert tracker.compute_latency_s == pytest.approx(0.040, abs=0.005)
        assert tracker.total_lead_s == pytest.approx(0.090, abs=0.005)

    def test_implausible_samples_are_discarded(self):
        tracker = LatencyTracker(initial_estimate_s=0.050, alpha=0.5)
        tracker.record(captured_at=10.0, commanded_at=5.0)     # negative
        tracker.record(captured_at=0.0, commanded_at=30.0)     # absurd
        assert tracker.compute_latency_s == pytest.approx(0.050)
        assert tracker.sample_count == 0

    def test_rejects_invalid_alpha(self):
        with pytest.raises(ValueError):
            LatencyTracker(alpha=0.0)


class TestSnapshotHolder:
    def test_newest_wins(self):
        holder = SnapshotHolder()
        holder.publish(TrackSnapshot(frame_id=1, captured_at=0.0, processed_at=0.1))
        holder.publish(TrackSnapshot(frame_id=2, captured_at=0.1, processed_at=0.2))
        assert holder.latest().frame_id == 2

    def test_clear_drops_stale_target(self):
        holder = SnapshotHolder()
        holder.publish(TrackSnapshot(frame_id=1, captured_at=0.0, processed_at=0.1))
        holder.clear()
        assert holder.latest() is None

    def test_pipeline_latency_is_computed(self):
        snapshot = TrackSnapshot(frame_id=1, captured_at=1.000, processed_at=1.032)
        assert snapshot.pipeline_latency_s == pytest.approx(0.032)

    def test_empty_snapshot_has_no_target(self):
        assert TrackSnapshot(frame_id=1, captured_at=0.0, processed_at=0.0).has_target is False
