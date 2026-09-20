"""Killswitch — SDDE Counter-UAS node entry point.

Wiring and the state machine loop. All domain logic lives in helper/; this file
constructs, sequences and tears down.

Read architecture.md section 5 for the engagement lifecycle and guardrails.md
section 2 before changing anything on the effector path.

Usage:
    python main.py --config config/bench.yaml
    python main.py --config config/bench.yaml --mock      # no hardware
"""
from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading
import time
from typing import Any, Dict, Optional, Tuple

import yaml

from helper.comms.audit import AuditLog
from helper.comms.mqtt_client import C2Client, MockC2Client
from helper.comms.video import VideoPublisher
from helper.comms.schemas import OperatorAuth, SlewToCue
from helper.hardware.actuator import ActuatorDriver, ActuatorError, MockActuator, SerialActuator
from helper.state.machine import (
    EngagementState,
    IllegalTransitionError,
    SnapshotHolder,
    StateMachine,
    TrackSnapshot,
    Transition,
)
from helper.state.control import ControlGains, compute_correction
from helper.state.sweep import SweepController, cue_to_gimbal
from helper.vision.aimpoint import AimpointSolver, error_magnitude, select_priority_target
from helper.vision.detector import Detector, DetectorError, ScriptedDetector, UltralyticsDetector
from helper.vision.frame_source import FrameSource, MockFrameSource, UsbCameraSource
from helper.vision.predictor import AimpointPredictor, LatencyTracker

logger = logging.getLogger("killswitch")


class KillswitchNode:
    """The targeting brain.

    Thread topology (see architecture.md section 2):

        main               state machine tick loop; the ONLY writer of state
        frame-grabber      inside UsbCameraSource; newest-frame-wins
        vision-worker      detection + tracking + aimpoint; publishes snapshots
        mqtt-network       paho; validates and enqueues, never acts
        serial-reader      inside SerialActuator; parses status frames
        serial-heartbeat   inside SerialActuator; refreshes the firmware deadman

    Cross-thread state moves through exactly three guarded objects: StateMachine
    (RLock), SnapshotHolder (Lock), and the C2 inbox (Queue). There is no shared
    mutable dict — that pattern is what this refactor removed.
    """

    def __init__(self, config: Dict[str, Any], mock: bool = False) -> None:
        self._cfg = config
        self._mock = mock

        # Dependencies are declared here and CONSTRUCTED in start(). Nothing is
        # implicitly available; nothing starts before it is injected.
        self._camera: Optional[FrameSource] = None
        self._detector: Optional[Detector] = None
        self._actuator: Optional[ActuatorDriver] = None
        self._c2 = None

        self._machine = StateMachine(on_transition=self._publish_transition)
        self._snapshots = SnapshotHolder()
        self._solver = AimpointSolver(
            offset_x=config["targeting"]["offset_x"],
            offset_y=config["targeting"]["offset_y"],
            min_box_px=config["targeting"]["min_box_px"],
        )
        self._predictor = AimpointPredictor(
            smoothing=config["prediction"]["smoothing"],
            max_lead_px=config["prediction"]["max_lead_px"],
        )
        self._latency = LatencyTracker(
            initial_estimate_s=config["prediction"]["initial_compute_latency_s"],
            mechanical_allowance_s=config["prediction"]["mechanical_allowance_s"],
        )
        self._sweep = SweepController(
            pan_step_deg=config["scan"]["pan_step_deg"],
            tilt_step_deg=config["scan"]["tilt_step_deg"],
            max_cycles=config["scan"]["max_cycles"],
        )

        self._running = threading.Event()
        self._vision_thread: Optional[threading.Thread] = None
        self._audit = AuditLog(node_id=config["node"]["id"])
        self._video: Optional[VideoPublisher] = None

        # Engagement bookkeeping. Owned by the main thread only.
        self._commanded: Tuple[float, float] = (
            config["actuator"]["stow_pan_deg"],
            config["actuator"]["stow_tilt_deg"],
        )
        self._hold_started_at: Optional[float] = None
        self._last_target_seen: float = 0.0
        self._active_track_id: Optional[int] = None
        self._active_target_id: Optional[str] = None
        self._burn_started_at: Optional[float] = None
        self._consumed_nonces: set = set()
        self._gains: Optional[ControlGains] = None
        self._gimbal_rate: Tuple[float, float] = (0.0, 0.0)
        self._last_command_at: Optional[float] = None
        self._engagement_metrics: Dict[str, Any] = {}

    # ---- lifecycle -------------------------------------------------------

    def start(self) -> None:
        """Construct and connect every dependency, THEN start worker threads.

        Ordering is the fix for the legacy init race, where the model thread was
        started at main.py:187 and immediately called change_state(IDLE), which
        dereferenced global_data["board"] — assigned 24 lines later at :211. It
        survived only because ONNX loading was slow enough to lose the race.

        Here nothing is started until everything it touches exists and is
        connected, and the vision worker asserts its dependencies before its
        first iteration.

        Raises:
            RuntimeError: If any dependency fails to initialise. Partial startup
                always tears down rather than running degraded.
        """
        logger.info("Starting node (mock=%s)", self._mock)
        try:
            self._build_actuator()      # 1. effector first: safe state established
            self._build_camera()        # 2. sensor
            self._build_detector()      # 3. model
            self._build_c2()            # 4. comms
        except Exception:
            logger.critical("Startup failed; tearing down")
            self.shutdown()
            raise

        assert self._camera is not None and self._detector is not None
        assert self._actuator is not None and self._c2 is not None

        width, _ = self._camera.frame_size
        self._gains = ControlGains.from_config(self._cfg, width)
        logger.info(
            "Control gain: %.4f deg/px (FOV %.1f over %d px), Kp=%.2f",
            self._gains.deg_per_px, self._cfg["camera"]["horizontal_fov_deg"],
            width, self._gains.proportional_gain,
        )

        if self._cfg.get("video", {}).get("enabled", True):
            video_cfg = self._cfg.get("video", {})
            self._video = VideoPublisher(
                self._c2,
                fps=video_cfg.get("fps", 15.0),
                max_width=video_cfg.get("max_width", 640),
                quality=video_cfg.get("quality", 60),
            )

        self._audit.write("node_start", {
            "mock": self._mock,
            "frame_size": list(self._camera.frame_size),
            "deg_per_px": round(self._gains.deg_per_px, 5),
        })

        # Only now do threads start.
        self._running.set()
        self._vision_thread = threading.Thread(
            target=self._vision_loop, name="vision-worker", daemon=True
        )
        self._vision_thread.start()
        self._enter_idle("startup")

    def _build_actuator(self) -> None:
        if self._mock:
            self._actuator = MockActuator()
        else:
            self._actuator = SerialActuator(
                port=self._cfg["actuator"]["port"], baud=self._cfg["actuator"]["baud"]
            )
        self._actuator.connect()

    def _build_camera(self) -> None:
        cam = self._cfg["camera"]
        if self._mock:
            self._camera = MockFrameSource(cam["width"], cam["height"])
        else:
            self._camera = UsbCameraSource(
                device_index=cam["device_index"],
                width=cam["width"],
                height=cam["height"],
                target_fps=cam["target_fps"],
                use_mjpg=cam["use_mjpg"],
            )
        self._camera.start()

    def _build_detector(self) -> None:
        det = self._cfg["detector"]
        if self._mock:
            self._detector = ScriptedDetector([])
            return
        self._detector = UltralyticsDetector(
            weights_path=det["weights"],
            conf_threshold=det["conf_threshold"],
            iou_threshold=det["iou_threshold"],
            target_classes=det["target_classes"],
            imgsz=det["imgsz"],
            device=det["device"],
        )

    def _build_c2(self) -> None:
        c2 = self._cfg["c2"]
        if self._mock:
            self._c2 = MockC2Client(auth_token=c2["auth_token"])
        else:
            self._c2 = C2Client(
                broker_host=c2["broker_host"],
                broker_port=c2["broker_port"],
                node_id=self._cfg["node"]["id"],
                auth_token=c2["auth_token"],
            )
        self._c2.connect()

    def shutdown(self) -> None:
        """De-energise, stop threads, release everything. Safe to call twice."""
        logger.info("Shutting down")
        self._running.clear()

        # Effector first, always.
        if self._actuator is not None:
            try:
                self._actuator.emergency_stop()
                if not self._actuator.confirm_effector_off(timeout=0.5):
                    logger.critical("COULD NOT CONFIRM EFFECTOR OFF during shutdown")
            except ActuatorError as exc:
                logger.critical("Actuator failed during shutdown: %s", exc)

        if self._vision_thread is not None and self._vision_thread.is_alive():
            self._vision_thread.join(timeout=2.0)

        # Each component exposes its own teardown verb; a generic closer
        # silently picked the wrong one and raised during shutdown.
        for component, verb in (
            (self._camera, "stop"),
            (self._c2, "disconnect"),
            (self._actuator, "close"),
        ):
            if component is None:
                continue
            try:
                getattr(component, verb)()
            except Exception as exc:  # noqa: BLE001 - teardown must not abort
                logger.error("Error during %s.%s(): %s", type(component).__name__, verb, exc)

        logger.info("Latency at shutdown: %s", self._latency.summary())
        self._audit.write("node_stop", {"latency": self._latency.summary()})
        if self._audit.path is not None:
            logger.info("Audit trail written to %s", self._audit.path)
        self._audit.close()

    # ---- vision worker ---------------------------------------------------

    def _vision_loop(self) -> None:
        """Detect, track and solve; publish snapshots. Runs on vision-worker.

        Decoupled from the tick loop so an 80 ms inference cannot delay a
        fail-safe check. The state machine reads whatever the most recent
        snapshot is and reasons about its age explicitly.
        """
        if self._camera is None or self._detector is None:
            logger.critical("Vision worker started without dependencies; aborting")
            return

        last_frame_id = -1
        while self._running.is_set():
            frame, frame_id = self._camera.read()
            if frame is None or frame_id == last_frame_id:
                time.sleep(0.002)
                continue
            last_frame_id = frame_id
            captured_at = time.monotonic()

            try:
                detections = self._detector.detect(frame)
            except DetectorError as exc:
                logger.error("Detector fault: %s", exc)
                self._machine.force_idle(f"detector fault: {exc}")
                time.sleep(0.1)
                continue

            center = self._camera.frame_center
            target = select_priority_target(detections, center)
            aim = self._solver.solve(target) if target is not None else None
            error = error_magnitude(aim, center) if aim is not None else float("inf")

            self._snapshots.publish(
                TrackSnapshot(
                    frame_id=frame_id,
                    captured_at=captured_at,
                    processed_at=time.monotonic(),
                    detections=tuple(detections),
                    target=target,
                    aim=aim,
                    error_px=error,
                )
            )

    # ---- main tick loop --------------------------------------------------

    def run(self) -> None:
        """Tick the state machine until stopped."""
        period = 1.0 / float(self._cfg["node"]["tick_hz"])
        handlers = {
            EngagementState.IDLE: self._tick_idle,
            EngagementState.SCAN: self._tick_scan,
            EngagementState.TRACK: self._tick_track,
            EngagementState.HOLD: self._tick_hold,
            EngagementState.OPERATOR_AUTH: self._tick_operator_auth,
            EngagementState.ENGAGE: self._tick_engage,
        }

        while self._running.is_set():
            tick_start = time.monotonic()
            try:
                self._check_liveness()
                handlers[self._machine.state]()
            except IllegalTransitionError as exc:
                logger.error("Illegal transition: %s", exc)
                self._enter_idle(f"illegal transition: {exc}")
            except ActuatorError as exc:
                logger.error("Actuator fault: %s", exc)
                self._enter_idle(f"actuator fault: {exc}")
            except Exception as exc:  # noqa: BLE001 - top-level guard
                # De-energise FIRST, then handle. Guardrails section 2, HARD.
                logger.critical("Unhandled fault in tick: %s", exc, exc_info=True)
                self._enter_idle(f"unhandled fault: {exc}")

            self._pump_video()

            elapsed = time.monotonic() - tick_start
            if elapsed < period:
                time.sleep(period - elapsed)

    def _pump_video(self) -> None:
        """Publish an annotated frame to the operator console. Best-effort."""
        if self._video is None or self._camera is None:
            return
        frame, _ = self._camera.read()
        if frame is None:
            return
        progress = 0.0
        if self._hold_started_at is not None:
            progress = (
                (time.monotonic() - self._hold_started_at)
                / self._cfg["engagement"]["hold_duration_s"]
            )
        self._video.maybe_publish(
            frame, self._snapshots.latest(), self._machine.state.value, progress
        )

    def _check_liveness(self) -> None:
        """Fault to IDLE on loss of any subsystem while past IDLE."""
        if self._machine.state is EngagementState.IDLE:
            return
        if self._c2 is not None and not self._c2.is_connected:
            self._enter_idle("C2 link lost")
        elif isinstance(self._actuator, SerialActuator) and not self._actuator.is_healthy:
            self._enter_idle("serial link lost")
        elif isinstance(self._camera, UsbCameraSource) and not self._camera.is_healthy:
            self._enter_idle("camera lost")

    # ---- state handlers --------------------------------------------------

    def _tick_idle(self) -> None:
        """Await a validated radar cue."""
        message = self._c2.poll()
        if not isinstance(message, SlewToCue):
            return

        gimbal = cue_to_gimbal(
            message.azimuth,
            message.elevation,
            boresight_az_deg=self._cfg["scan"]["boresight_azimuth_deg"],
        )
        if gimbal is None:
            self._c2.publish_event(
                "cue_rejected",
                {"target_id": message.target_id, "reason": "outside gimbal arc"},
            )
            return

        pan, tilt = gimbal
        self._active_target_id = message.target_id
        self._sweep.reset(pan, tilt)
        self._command_gimbal(pan, tilt)
        self._machine.transition_to(
            EngagementState.SCAN,
            f"cue {message.target_id} az={message.azimuth:.1f} el={message.elevation:.1f}",
        )

    def _tick_scan(self) -> None:
        """Sweep the cued volume looking for a visual lock."""
        snapshot = self._snapshots.latest()
        if snapshot is not None and snapshot.has_target:
            self._begin_track(snapshot)
            return

        if self._machine.time_in_state > self._cfg["engagement"]["scan_timeout_s"]:
            self._enter_idle("scan timeout, no visual acquired")
            return
        if self._sweep.exhausted:
            self._enter_idle(f"search volume covered {self._sweep.cycles_completed}x")
            return

        pan, tilt = self._sweep.step()
        self._command_gimbal(pan, tilt)

    def _tick_track(self) -> None:
        """Drive the aimpoint to boresight."""
        snapshot = self._drive_to_target()
        if snapshot is None:
            return
        if snapshot.error_px <= self._cfg["engagement"]["hold_error_px"]:
            self._hold_started_at = time.monotonic()
            self._machine.transition_to(
                EngagementState.HOLD, f"error {snapshot.error_px:.1f}px inside band"
            )

    def _tick_hold(self) -> None:
        """Hold the aimpoint inside the error band for the required duration."""
        snapshot = self._drive_to_target()
        if snapshot is None:
            self._hold_started_at = None
            return

        # Any excursion resets the timer. It never accumulates across breaks
        # (guardrails section 6). An identity switch is also an excursion: a new
        # track id is a different object until proven otherwise.
        if snapshot.error_px > self._cfg["engagement"]["hold_error_px"]:
            self._hold_started_at = None
            self._machine.transition_to(
                EngagementState.TRACK, f"error {snapshot.error_px:.1f}px left band"
            )
            return
        if snapshot.aim is not None and snapshot.aim.track_id != self._active_track_id:
            self._hold_started_at = None
            self._active_track_id = snapshot.aim.track_id
            self._machine.transition_to(EngagementState.TRACK, "track identity switched")
            return

        if self._hold_started_at is None:
            self._hold_started_at = time.monotonic()
            return

        held = time.monotonic() - self._hold_started_at
        if held >= self._cfg["engagement"]["hold_duration_s"]:
            self._publish_firing_solution(snapshot, held)
            self._machine.transition_to(
                EngagementState.OPERATOR_AUTH, f"held {held:.2f}s within band"
            )

    def _tick_operator_auth(self) -> None:
        """Await the operator decision. Keep tracking — the target does not wait."""
        snapshot = self._drive_to_target()
        if snapshot is None:
            return

        if self._machine.time_in_state > self._cfg["engagement"]["auth_timeout_s"]:
            self._machine.transition_to(EngagementState.TRACK, "operator auth timed out")
            return

        message = self._c2.poll()
        if not isinstance(message, OperatorAuth):
            return

        if message.nonce in self._consumed_nonces:
            logger.warning("Replayed auth nonce %s rejected", message.nonce)
            return
        self._consumed_nonces.add(message.nonce)

        if message.target_id != self._active_target_id:
            logger.warning(
                "Auth for %s does not match active target %s; rejected",
                message.target_id, self._active_target_id,
            )
            return

        if not message.auth:
            self._enter_idle(f"operator DENIED engagement on {message.target_id}")
            return

        self._begin_engagement(snapshot)

    def _tick_engage(self) -> None:
        """Burn for the configured duration, tracking throughout."""
        if self._burn_started_at is None:
            self._end_engagement("burn state entered without a start time")
            return

        elapsed = time.monotonic() - self._burn_started_at
        snapshot = self._snapshots.latest()

        # Lock loss during a burn cuts the beam immediately. Do not coast.
        if snapshot is None or not snapshot.has_target:
            self._end_engagement(f"lock lost {elapsed:.2f}s into burn")
            return

        self._drive_to_target()
        self._engagement_metrics["peak_error_px"] = max(
            self._engagement_metrics.get("peak_error_px", 0.0), snapshot.error_px
        )
        self._engagement_metrics["samples"] = self._engagement_metrics.get("samples", 0) + 1
        self._engagement_metrics["error_sum"] = (
            self._engagement_metrics.get("error_sum", 0.0) + snapshot.error_px
        )

        if elapsed >= self._cfg["engagement"]["burn_duration_s"]:
            self._end_engagement(f"burn complete at {elapsed:.2f}s")

    # ---- shared mechanics ------------------------------------------------

    def _drive_to_target(self) -> Optional[TrackSnapshot]:
        """Command the gimbal toward the predicted aimpoint.

        Returns:
            The snapshot acted on, or None if the target is missing — in which
            case this method has already handled the track-loss transition.
        """
        snapshot = self._snapshots.latest()
        now = time.monotonic()

        if snapshot is None or not snapshot.has_target or snapshot.aim is None:
            if now - self._last_target_seen > self._cfg["engagement"]["track_loss_timeout_s"]:
                if self._machine.state is not EngagementState.SCAN:
                    self._predictor.reset()
                    self._hold_started_at = None
                    self._machine.transition_to(
                        EngagementState.SCAN, "target lost, reacquiring"
                    )
            return None

        self._last_target_seen = now
        self._active_track_id = snapshot.aim.track_id
        self._predictor.update(snapshot.aim, snapshot.processed_at)
        # Sample latency for every processed snapshot, not only for those that
        # produce a command: a target sitting inside the deadband would
        # otherwise never update the estimate that drives lead prediction.
        self._latency.record(snapshot.captured_at, now)

        predicted = self._predictor.predict(self._latency.total_lead_s)
        aim_x, aim_y = predicted if predicted is not None else (snapshot.aim.x, snapshot.aim.y)

        # Velocity feed-forward. A pure P controller needs a standing error to
        # sustain a slew rate, which puts a fast target permanently outside the
        # HOLD band. The target's world rate is its apparent image rate PLUS the
        # gimbal's own rate — without the second term the estimate collapses to
        # zero exactly when tracking starts working. See helper/state/control.py.
        dt = (now - self._last_command_at) if self._last_command_at else 0.0
        self._last_command_at = now
        feedforward = (0.0, 0.0)
        velocity = self._predictor.velocity_px_s
        if velocity is not None and 0.0 < dt < 0.5:
            px_per_deg = 1.0 / self._gains.deg_per_px
            world_pan_rate = velocity[0] / px_per_deg + self._gimbal_rate[0]
            world_tilt_rate = -velocity[1] / px_per_deg + self._gimbal_rate[1]
            feedforward = (world_pan_rate * dt, world_tilt_rate * dt)

        # Same function the closed-loop simulator exercises.
        correction = compute_correction(
            aim_x, aim_y, self._camera.frame_center, self._gains,
            feedforward_deg=feedforward,
        )
        if correction is None:
            self._gimbal_rate = (0.0, 0.0)
            return snapshot

        delta_pan, delta_tilt = correction
        self._gimbal_rate = (delta_pan / dt, delta_tilt / dt) if dt > 0 else (0.0, 0.0)
        self._command_gimbal(self._commanded[0] + delta_pan, self._commanded[1] + delta_tilt)
        return snapshot

    def _command_gimbal(self, pan: float, tilt: float) -> None:
        """Send absolute angles. Clamped in the driver AND again in firmware."""
        self._actuator.set_angles(pan, tilt)
        status = self._actuator.last_status()
        self._commanded = (status.pan, status.tilt) if status is not None else (pan, tilt)

    def _begin_track(self, snapshot: TrackSnapshot) -> None:
        self._last_target_seen = time.monotonic()
        self._active_track_id = snapshot.aim.track_id if snapshot.aim else None
        self._predictor.reset()
        self._machine.transition_to(
            EngagementState.TRACK,
            f"visual acquired, {len(snapshot.detections)} detection(s)",
        )

    def _begin_engagement(self, snapshot: TrackSnapshot) -> None:
        """Arm, energise, start the burn clock."""
        self._actuator.arm()
        self._actuator.set_effector(True)
        self._burn_started_at = time.monotonic()
        self._engagement_metrics = {
            "target_id": self._active_target_id,
            "track_id": self._active_track_id,
            "started_at": self._burn_started_at,
            "lead_ms": self._latency.total_lead_s * 1000.0,
        }
        self._machine.transition_to(
            EngagementState.ENGAGE, f"operator AUTHORISED {self._active_target_id}"
        )

    def _end_engagement(self, reason: str) -> None:
        """De-energise, confirm, log metrics, return to IDLE."""
        elapsed = time.monotonic() - self._burn_started_at if self._burn_started_at else 0.0
        self._actuator.set_effector(False)
        self._actuator.disarm()

        if not self._actuator.confirm_effector_off(timeout=0.5):
            logger.critical("EFFECTOR OFF NOT CONFIRMED after burn — treating as fault")

        samples = self._engagement_metrics.get("samples", 0)
        metrics = {
            **self._engagement_metrics,
            "reason": reason,
            "time_on_target_s": round(elapsed, 3),
            "mean_error_px": round(
                self._engagement_metrics.get("error_sum", 0.0) / samples, 2
            ) if samples else None,
            "peak_error_px": round(self._engagement_metrics.get("peak_error_px", 0.0), 2),
            "measured_compute_latency_ms": round(self._latency.compute_latency_s * 1000.0, 1),
        }
        logger.info("ENGAGEMENT COMPLETE: %s", metrics)
        self._audit.write("engagement_complete", metrics)
        self._c2.publish_event("engagement_complete", metrics)

        self._burn_started_at = None
        self._enter_idle(reason)

    def _enter_idle(self, reason: str) -> None:
        """Universal safe harbour. Never raises — every fault path ends here."""
        try:
            if self._actuator is not None:
                self._actuator.set_effector(False)
                self._actuator.disarm()
                self._actuator.set_angles(
                    self._cfg["actuator"]["stow_pan_deg"],
                    self._cfg["actuator"]["stow_tilt_deg"],
                )
        except ActuatorError as exc:
            logger.critical(
                "Could not safe the actuator entering IDLE: %s. Firmware deadman "
                "will cut the effector within its window.", exc,
            )

        self._hold_started_at = None
        self._burn_started_at = None
        self._active_track_id = None
        self._active_target_id = None
        self._gimbal_rate = (0.0, 0.0)
        self._last_command_at = None
        self._predictor.reset()
        self._snapshots.clear()
        if self._detector is not None:
            self._detector.reset()
        if self._c2 is not None:
            self._c2.drain()

        if self._machine.state is EngagementState.IDLE:
            return
        # Prefer the declared transition so the audit trail distinguishes a
        # nominal return to IDLE from a fault. force_idle is the fault path.
        try:
            self._machine.transition_to(EngagementState.IDLE, reason)
        except IllegalTransitionError:
            self._machine.force_idle(reason)

    def _publish_firing_solution(self, snapshot: TrackSnapshot, held: float) -> None:
        aim = snapshot.aim
        self._c2.publish_event(
            "firing_solution",
            {
                "target_id": self._active_target_id,
                "track_id": aim.track_id if aim else None,
                "error_px": round(snapshot.error_px, 2),
                "hold_s": round(held, 2),
                "aimpoint_downgraded": aim.downgraded if aim else None,
                "aimpoint_reason": aim.reason if aim else None,
                "lead_ms": round(self._latency.total_lead_s * 1000.0, 1),
                "target_speed_px_s": round(self._predictor.speed_px_s, 1),
            },
        )

    def _publish_transition(self, transition: Transition) -> None:
        """StateMachine callback. Must not block — publish is QoS 1, non-blocking."""
        detail = {
            "from": transition.from_state.value,
            "to": transition.to_state.value,
            "reason": transition.reason,
        }
        self._audit.write("state_transition", detail)
        if self._c2 is not None:
            self._c2.publish_event("state_transition", detail)

    @property
    def state(self) -> EngagementState:
        return self._machine.state

    def stop(self) -> None:
        self._running.clear()


def load_config(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/bench.yaml")
    parser.add_argument("--mock", action="store_true", help="Run with no hardware")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    node = KillswitchNode(load_config(args.config), mock=args.mock)

    def handle_signal(signum, _frame):
        logger.info("Signal %d received, stopping", signum)
        node.stop()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        node.start()
        node.run()
    except (RuntimeError, ActuatorError, ConnectionError) as exc:
        logger.critical("Fatal: %s", exc)
        return 1
    finally:
        node.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
