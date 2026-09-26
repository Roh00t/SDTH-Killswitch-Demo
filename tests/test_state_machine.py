"""State machine, sweep and prediction tests. No hardware, no model, no broker."""
from __future__ import annotations

import time

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

    def test_snapshot_pairs_state_with_its_own_age(self):
        machine = StateMachine()
        advance(machine, S.SCAN, S.TRACK, S.HOLD, S.OPERATOR_AUTH)
        machine._entered_at -= 4.0
        state, age = machine.snapshot()
        assert state is S.OPERATOR_AUTH
        assert age == pytest.approx(4.0, abs=0.05)

        machine.force_idle("fault")
        state, age = machine.snapshot()
        assert state is S.IDLE
        assert age < 1.0, "a transition must restart the age with the state"


class TestAuthCountdownTelemetry:
    """Panel C shows the decision window in OPERATOR_AUTH, not the hold timer.

    The hold timer keeps running from when HOLD began, so the dashboard read
    "12.1 / 3.0 s" for the whole auth window, which looks like a fault.
    """

    @staticmethod
    def _node_in(*path):
        from main import KillswitchNode, _RateGate, load_config

        node = KillswitchNode(
            load_config("config/fallback.yaml"), sim_target=True, mock_c2=True
        )
        node._build_c2()                            # MockC2Client: records publishes
        node._telemetry_limiter = _RateGate(hz=0)   # every call publishes
        advance(node._machine, *path)
        return node

    @staticmethod
    def _last_telemetry(node):
        node._pump_telemetry()
        kind, state, detail = node._c2.published[-1]
        assert kind == "telemetry"
        return state, detail

    def test_absent_outside_the_auth_window(self):
        node = self._node_in(S.SCAN, S.TRACK, S.HOLD)
        state, detail = self._last_telemetry(node)
        assert state == "HOLD"
        assert detail["auth_remaining_s"] is None
        assert detail["auth_timeout_s"] == node._cfg["engagement"]["auth_timeout_s"]

    def test_counts_down_through_the_window(self):
        node = self._node_in(S.SCAN, S.TRACK, S.HOLD, S.OPERATOR_AUTH)
        timeout = node._cfg["engagement"]["auth_timeout_s"]

        state, detail = self._last_telemetry(node)
        assert state == "OPERATOR_AUTH"
        assert detail["auth_remaining_s"] == pytest.approx(timeout, abs=0.05)

        node._machine._entered_at -= 7.5
        _, detail = self._last_telemetry(node)
        assert detail["auth_remaining_s"] == pytest.approx(timeout - 7.5, abs=0.05)

    def test_hold_time_is_reported_only_while_holding(self):
        node = self._node_in(S.SCAN, S.TRACK, S.HOLD)
        node._hold_started_at = time.monotonic() - 1.5
        _, detail = self._last_telemetry(node)
        assert detail["hold_s"] == pytest.approx(1.5, abs=0.05)

        # The timer is not reset on the way into OPERATOR_AUTH, or on the auth
        # timeout back to TRACK. Neither state may report it as hold progress.
        node._hold_started_at = time.monotonic() - 13.0
        for state in (S.OPERATOR_AUTH, S.TRACK):
            node._machine.transition_to(state, "test")
            _, detail = self._last_telemetry(node)
            assert detail["hold_s"] == 0.0, state

    def test_never_negative_once_the_window_has_passed(self):
        node = self._node_in(S.SCAN, S.TRACK, S.HOLD, S.OPERATOR_AUTH)
        node._machine._entered_at -= node._cfg["engagement"]["auth_timeout_s"] + 3.0
        _, detail = self._last_telemetry(node)
        assert detail["auth_remaining_s"] == 0.0


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


class TestFailSafeGaps:
    """Two paths that bypassed the safe harbour.

    A force_idle from the vision thread left the effector on until the
    firmware burn ceiling, and an auth that missed its window fired the next
    one with no fresh decision.
    """

    @staticmethod
    def _node(actuator=None):
        from helper.hardware.actuator import MockActuator
        from main import KillswitchNode, load_config

        node = KillswitchNode(
            load_config("config/fallback.yaml"), sim_target=True, mock_c2=True
        )
        node._build_c2()
        node._actuator = actuator or MockActuator()
        node._actuator.connect()
        node._audit_records = []
        node._audit.write = lambda event, detail: node._audit_records.append((event, detail))
        return node

    @staticmethod
    def _burning(node):
        advance(node._machine, S.SCAN, S.TRACK, S.HOLD, S.OPERATOR_AUTH, S.ENGAGE)
        node._actuator.arm()
        node._actuator.set_effector(True)
        node._resafe_if_forced_idle()          # a tick in ENGAGE clears the flag
        assert node._actuator.effector_on

    def test_forced_idle_mid_burn_is_safed_within_one_tick(self):
        node = self._node()
        self._burning(node)

        node._machine.force_idle("detector fault: test")   # the vision thread's call
        node._resafe_if_forced_idle()

        act, cfg = node._actuator, node._cfg["actuator"]
        assert not act.effector_on and not act.armed
        assert (act.pan, act.tilt) == (cfg["stow_pan_deg"], cfg["stow_tilt_deg"])
        assert node._idle_safed
        assert node._audit_records[-1] == (
            "forced_idle_safed", {"cause": "FORCED: detector fault: test", "safed": True})

    def test_safed_idle_is_not_safed_again(self):
        node = self._node()
        self._burning(node)
        node._machine.force_idle("detector fault: test")
        node._resafe_if_forced_idle()
        commands = len(node._actuator.history)
        node._resafe_if_forced_idle()
        assert len(node._actuator.history) == commands

    def test_failed_safing_retries_at_the_deadman_period(self):
        from helper.hardware.actuator import ActuatorError, MockActuator
        from helper.hardware.protocol import DEADMAN_TIMEOUT_MS

        class Flaky(MockActuator):
            failures = 1

            def set_effector(self, on):
                if not on and self.failures:
                    self.failures -= 1
                    raise ActuatorError("serial write failed")
                super().set_effector(on)

        node = self._node(Flaky())
        self._burning(node)
        node._machine.force_idle("detector fault: test")

        node._resafe_if_forced_idle()
        assert not node._idle_safed and node._actuator.effector_on
        assert node._next_safe_attempt_at - time.monotonic() == pytest.approx(
            DEADMAN_TIMEOUT_MS / 1000.0, abs=0.05)

        node._resafe_if_forced_idle()          # not due yet: no second attempt
        assert node._actuator.effector_on

        node._next_safe_attempt_at = 0.0       # the deadman period has passed
        node._resafe_if_forced_idle()
        assert node._idle_safed and not node._actuator.effector_on

    def test_stale_auth_is_discarded_when_the_window_opens(self):
        from helper.comms.schemas import OperatorAuth

        node = self._node()
        advance(node._machine, S.SCAN, S.TRACK, S.HOLD)
        node._active_target_id = "TRK-BETA_01"
        # SPACE that landed after the previous window had already timed out.
        node._c2.inject(OperatorAuth(auth=True, target_id="TRK-BETA_01",
                                     token="sdth-demo-token", nonce="late"))

        node._open_auth_window(TrackSnapshot(frame_id=1, captured_at=0.0,
                                             processed_at=0.0, error_px=2.0), 3.0)

        assert node._machine.state is S.OPERATOR_AUTH
        assert node._c2.poll() is None, "a pre-window auth must not reach this window"

    def test_a_decision_inside_the_window_still_arrives(self):
        from helper.comms.schemas import OperatorAuth

        node = self._node()
        advance(node._machine, S.SCAN, S.TRACK, S.HOLD)
        node._open_auth_window(TrackSnapshot(frame_id=1, captured_at=0.0,
                                             processed_at=0.0, error_px=2.0), 3.0)
        auth = OperatorAuth(auth=True, target_id="TRK-BETA_01",
                            token="sdth-demo-token", nonce="in-window")
        node._c2.inject(auth)
        assert node._c2.poll() == auth


class TestStepAndStare:
    """SCAN moves, stops and looks. It used to step every 10 ms tick (400 deg/s)."""

    @staticmethod
    def _scanning():
        node = TestFailSafeGaps._node()
        advance(node._machine, S.SCAN)
        node._sweep.reset(45.0, 97.0)
        node._stare_at(45.0, 97.0)
        return node

    @staticmethod
    def _look(node, frame_id, after_settle=True):
        from helper.state.machine import TrackSnapshot

        settle = node._cfg["scan"]["settle_s"]
        captured = node._stare_since + (settle + 0.01 if after_settle else settle / 2)
        node._snapshots.publish(TrackSnapshot(
            frame_id=frame_id, captured_at=captured, processed_at=captured + 0.05))

    def test_holds_still_until_it_has_looked(self):
        node = self._scanning()
        for _ in range(50):                    # half a second of ticks, no frames
            node._tick_scan()
        assert (node._actuator.pan, node._actuator.tilt) == (45.0, 97.0)

    def test_frames_from_before_the_gimbal_settled_do_not_count(self):
        node = self._scanning()
        for frame_id in range(1, 6):
            self._look(node, frame_id, after_settle=False)
            node._tick_scan()
        assert node._actuator.pan == 45.0

    def test_the_same_frame_counts_once(self):
        node = self._scanning()
        self._look(node, 1)
        for _ in range(10):
            node._tick_scan()
        assert node._actuator.pan == 45.0

    def test_moves_one_step_after_enough_settled_looks_then_stops_again(self):
        node = self._scanning()
        step = node._cfg["scan"]["pan_step_deg"]
        for frame_id in range(1, node._cfg["scan"]["looks_per_stop"] + 1):
            self._look(node, frame_id)
            node._tick_scan()
        assert node._actuator.pan == 45.0 + step
        for _ in range(20):                    # a new stop: no new looks yet
            node._tick_scan()
        assert node._actuator.pan == 45.0 + step

    def test_a_target_seen_while_looking_starts_track(self):
        from helper.state.machine import TrackSnapshot

        node = self._scanning()
        target = det(320, 240)
        node._snapshots.publish(TrackSnapshot(
            frame_id=1, captured_at=node._stare_since + 0.6, processed_at=node._stare_since + 0.65,
            detections=(target,), target=target))
        node._tick_scan()
        assert node._machine.state is S.TRACK
