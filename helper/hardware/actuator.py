"""Actuator drivers — the gimbal and effector boundary.

Two implementations behind one ABC: SerialActuator (ESP32-S3 over the UART
bridge) and MockActuator (in-memory). Both enforce identical safety logic, so
the test suite exercises the real rules rather than a permissive stand-in.
"""
from __future__ import annotations

import logging
import threading
import time
from abc import ABC, abstractmethod
from typing import List, Optional, Tuple

from helper.hardware.protocol import (
    BAUD_RATE,
    ERROR_TEXT,
    HEARTBEAT_INTERVAL_S,
    ActuatorStatus,
    clamp_pan,
    clamp_tilt,
    frame,
)

logger = logging.getLogger(__name__)


class ActuatorError(RuntimeError):
    """Actuator fault. Callers transition to IDLE with the effector de-energised."""


class ActuatorDriver(ABC):
    """Gimbal plus effector contract.

    Safety invariant for every implementation: the effector is de-energised on
    construction, on close, on any error path, and whenever `arm` is False.
    """

    @abstractmethod
    def connect(self) -> None:
        """Open the link and bring the actuator to a known safe state."""

    @abstractmethod
    def set_angles(self, pan_deg: float, tilt_deg: float) -> None:
        """Command absolute angles. Clamped host-side before transmission."""

    @abstractmethod
    def arm(self) -> None:
        """Arm the effector subsystem. Required before `set_effector(True)`."""

    @abstractmethod
    def disarm(self) -> None:
        """Disarm. Also de-energises the effector."""

    @abstractmethod
    def set_effector(self, on: bool) -> None:
        """Energise or de-energise. `True` requires a prior `arm()`."""

    @abstractmethod
    def emergency_stop(self) -> None:
        """Immediate de-energise and disarm latch. Must never raise."""

    @abstractmethod
    def last_status(self) -> Optional[ActuatorStatus]:
        """Most recent status frame, or None if none received."""

    @abstractmethod
    def confirm_effector_off(self, timeout: float = 0.5) -> bool:
        """Block until a status frame confirms the effector is de-energised.

        Fire-and-forget is not adequate for an effector command (guardrails
        section 2, HARD). Returns False on timeout — the caller must treat that
        as a fault, not as success.
        """

    @abstractmethod
    def close(self) -> None:
        """De-energise, then release the link. Idempotent."""


class SerialActuator(ActuatorDriver):
    """ESP32-S3 gimbal over the UART bridge.

    Threading: owns a writer lock (one writer on the port, guardrails section 1
    HARD), a reader thread parsing inbound frames, and a heartbeat thread that
    refreshes the firmware deadman. `set_angles` and the arm/fire methods are
    safe from any thread.
    """

    def __init__(
        self,
        port: str,
        baud: int = BAUD_RATE,
        connect_timeout: float = 3.0,
    ) -> None:
        """Configure the link. Does not open it; call `connect()`.

        Args:
            port: Device path, e.g. '/dev/cu.usbserial-0001' (the UART bridge,
                not the native-USB CDC port — the bridge stays enumerated across
                ESP32 resets).
            baud: Line rate.
            connect_timeout: Seconds to wait for the firmware's first status.
        """
        self._port_name = port
        self._baud = baud
        self._connect_timeout = connect_timeout

        self._serial = None
        self._write_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._running = threading.Event()
        self._reader: Optional[threading.Thread] = None
        self._heartbeat: Optional[threading.Thread] = None

        self._last_status: Optional[ActuatorStatus] = None
        self._status_event = threading.Event()
        self._last_write: float = 0.0
        self._armed: bool = False
        self._errors: List[str] = []

    def connect(self) -> None:
        """Open the port, start threads, and force a known safe state.

        Raises:
            ActuatorError: If the port cannot be opened or firmware is silent.
        """
        try:
            import serial
        except ImportError as exc:
            raise ActuatorError("pyserial not installed. pip install pyserial") from exc

        try:
            self._serial = serial.Serial(
                self._port_name, self._baud, timeout=0.1, write_timeout=0.5
            )
        except (OSError, ValueError) as exc:
            raise ActuatorError(f"Could not open {self._port_name}: {exc}") from exc

        # ESP32 resets when the bridge asserts DTR; wait out the boot.
        time.sleep(2.0)
        self._serial.reset_input_buffer()

        self._running.set()
        self._reader = threading.Thread(
            target=self._reader_loop, name="serial-reader", daemon=True
        )
        self._reader.start()
        self._heartbeat = threading.Thread(
            target=self._heartbeat_loop, name="serial-heartbeat", daemon=True
        )
        self._heartbeat.start()

        # Known safe state before anything else touches the actuator.
        self.emergency_stop()

        if not self._status_event.wait(timeout=self._connect_timeout):
            self.close()
            raise ActuatorError(
                f"No status from firmware on {self._port_name} within "
                f"{self._connect_timeout}s. Check the board is running the "
                f"actuator sketch and that you are on the UART bridge port."
            )
        logger.info("Actuator connected on %s: %s", self._port_name, self._last_status)

    def _write(self, payload: bytes) -> None:
        """Single writer. Interleaved writes from two threads corrupt framing."""
        with self._write_lock:
            if self._serial is None:
                raise ActuatorError("Serial port is not open")
            try:
                self._serial.write(payload)
                self._last_write = time.monotonic()
            except (OSError, ValueError) as exc:
                raise ActuatorError(f"Serial write failed: {exc}") from exc

    def _reader_loop(self) -> None:
        """Parse inbound frames. Runs on the reader thread."""
        while self._running.is_set():
            port = self._serial
            if port is None:
                break
            try:
                raw = port.readline()
            except (OSError, ValueError) as exc:
                logger.error("Serial read failed: %s", exc)
                self._running.clear()
                break

            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if not line:
                continue

            status = ActuatorStatus.parse(line, time.monotonic())
            if status is not None:
                with self._state_lock:
                    self._last_status = status
                    self._armed = status.armed
                self._status_event.set()
                continue

            if line.startswith("ERR "):
                parts = line.split(maxsplit=2)
                code = parts[1] if len(parts) > 1 else "?"
                logger.error(
                    "Firmware %s: %s | %s", code,
                    ERROR_TEXT.get(code, "unrecognised code"), line,
                )
                with self._state_lock:
                    self._errors.append(line)
            elif not line.startswith("OK"):
                logger.debug("Firmware: %s", line)

    def _heartbeat_loop(self) -> None:
        """Refresh the firmware deadman when the loop is otherwise idle."""
        while self._running.is_set():
            time.sleep(HEARTBEAT_INTERVAL_S / 2.0)
            if time.monotonic() - self._last_write < HEARTBEAT_INTERVAL_S:
                continue
            try:
                self._write(frame("P"))
            except ActuatorError as exc:
                logger.error("Heartbeat failed, link is down: %s", exc)
                self._running.clear()
                break

    def set_angles(self, pan_deg: float, tilt_deg: float) -> None:
        """Command absolute angles, clamped host-side. See ActuatorDriver."""
        pan, tilt = clamp_pan(pan_deg), clamp_tilt(tilt_deg)
        self._write(frame(f"A{pan:05.1f},{tilt:05.1f}"))

    def arm(self) -> None:
        """Arm. Checksummed: hazard-increasing."""
        self._write(frame("M1", require_checksum=True))

    def disarm(self) -> None:
        """Disarm. Unchecksummed: must never be rejected."""
        self._write(frame("M0"))

    def set_effector(self, on: bool) -> None:
        """Energise/de-energise. `True` is checksummed and requires prior arm."""
        if on:
            self._write(frame("L1", require_checksum=True))
        else:
            self._write(frame("L0"))

    def emergency_stop(self) -> None:
        """Immediate kill. Never raises — this must work on every path."""
        try:
            self._write(frame("Z"))
        except ActuatorError as exc:
            logger.critical(
                "E-STOP COULD NOT BE SENT: %s. Firmware deadman will cut the "
                "effector within the timeout window.", exc,
            )

    def last_status(self) -> Optional[ActuatorStatus]:
        with self._state_lock:
            return self._last_status

    def confirm_effector_off(self, timeout: float = 0.5) -> bool:
        """Wait for a status frame showing the effector de-energised."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = self.last_status()
            if status is not None and not status.laser_on:
                # Require a frame generated after we asked.
                if status.received_at >= self._last_write - HEARTBEAT_INTERVAL_S:
                    return True
            time.sleep(0.02)
        logger.critical("Could not confirm effector OFF within %.2fs", timeout)
        return False

    @property
    def is_healthy(self) -> bool:
        """False once the link has faulted."""
        return self._running.is_set()

    def close(self) -> None:
        """De-energise, stop threads, release the port. Idempotent."""
        if self._serial is not None:
            self.emergency_stop()
            self.confirm_effector_off(timeout=0.3)

        self._running.clear()
        for thread in (self._reader, self._heartbeat):
            if thread is not None and thread.is_alive():
                thread.join(timeout=1.0)
        self._reader = self._heartbeat = None

        if self._serial is not None:
            self._serial.close()
            self._serial = None
        logger.info("Actuator link closed")


class MockActuator(ActuatorDriver):
    """In-memory actuator with identical safety logic.

    This is the second implementation that makes hardware agnosticism real
    rather than asserted, and it is what the test suite runs against.
    """

    def __init__(self, strict: bool = True) -> None:
        """Args:
            strict: Raise on a fire attempt while disarmed or e-stop latched,
                mirroring firmware rejection.
        """
        self._strict = strict
        self.pan: float = 90.0
        self.tilt: float = 90.0
        self.armed: bool = False
        self.effector_on: bool = False
        self.estop_latched: bool = False
        self.connected: bool = False
        self.history: List[Tuple[float, str]] = []
        self.rejections: List[str] = []
        self._t0 = time.monotonic()

    def _record(self, entry: str) -> None:
        self.history.append((time.monotonic() - self._t0, entry))

    def connect(self) -> None:
        self.connected = True
        self.emergency_stop()
        self._record("connect")

    def set_angles(self, pan_deg: float, tilt_deg: float) -> None:
        self.pan, self.tilt = clamp_pan(pan_deg), clamp_tilt(tilt_deg)
        self._record(f"A{self.pan:.1f},{self.tilt:.1f}")

    def arm(self) -> None:
        if self.estop_latched:
            self.rejections.append("arm while e-stop latched")
            if self._strict:
                raise ActuatorError("E08: e-stop latched, re-arm blocked")
            return
        self.armed = True
        self._record("M1")

    def disarm(self) -> None:
        self.armed = False
        self.effector_on = False
        self._record("M0")

    def set_effector(self, on: bool) -> None:
        if not on:
            self.effector_on = False
            self._record("L0")
            return
        if self.estop_latched:
            self.rejections.append("fire while e-stop latched")
            if self._strict:
                raise ActuatorError("E08: e-stop latched")
            return
        if not self.armed:
            self.rejections.append("fire while disarmed")
            if self._strict:
                raise ActuatorError("E04: not armed")
            return
        self.effector_on = True
        self._record("L1")

    def emergency_stop(self) -> None:
        self.effector_on = False
        self.armed = False
        self.estop_latched = True
        self._record("Z")

    def clear_estop(self) -> None:
        """Clear the latch. No firmware equivalent by design — a real e-stop is
        cleared by an operator, not by software. Test affordance only."""
        self.estop_latched = False
        self._record("clear_estop")

    def last_status(self) -> Optional[ActuatorStatus]:
        if not self.connected:
            return None
        return ActuatorStatus(
            pan=self.pan, tilt=self.tilt, laser_on=self.effector_on,
            armed=self.armed, uptime_ms=int((time.monotonic() - self._t0) * 1000),
            received_at=time.monotonic(),
        )

    def confirm_effector_off(self, timeout: float = 0.5) -> bool:
        return not self.effector_on

    def close(self) -> None:
        self.emergency_stop()
        self.connected = False
        self._record("close")
