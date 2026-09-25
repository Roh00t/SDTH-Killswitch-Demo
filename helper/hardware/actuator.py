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


# USB-UART bridge chips used on ESP32 devkits. Matched against the port
# description so the node survives the port number changing between plugs —
# macOS hands out /dev/cu.usbserial-<n> where <n> is not stable.
#
# "CH34" covers the whole WCH family (CH340/CH341/CH343/CH344/CH347) rather than
# one part number: the ESP32-S3-N16R8 boards ship a CH343, whose Windows driver
# reports "USB-Enhanced-SERIAL CH343 (COMn)" and which the CH340-only list
# silently failed to match — auto-detect then reported no bridge at all.
#
# Deliberately absent: "USB Serial", which is how the ESP32-S3's NATIVE USB-CDC
# peripheral enumerates on Windows ("USB Serial Device (COMn)"). That port
# re-enumerates on every ESP32 reset and takes the host's serial handle with it.
# Not matching it is the feature: auto-detect refusing the native port is
# cheaper to diagnose than a handle that dies mid-engagement.
_BRIDGE_HINTS = (
    "CP210", "CH34", "CH910", "FT232", "SLAB",
    "USB to UART", "USB-Enhanced-SERIAL", "USB-Serial",
)


def resolve_port(requested: str) -> str:
    """Resolve a configured port, auto-detecting when asked.

    Args:
        requested: A device path, or "auto" to search for a USB-UART bridge.

    Returns:
        A concrete device path.

    Raises:
        ActuatorError: If "auto" was requested and no bridge was found, or more
            than one was found (ambiguous — name it explicitly rather than
            guessing which board to drive).
    """
    if requested.lower() != "auto":
        return requested

    try:
        from serial.tools import list_ports
    except ImportError as exc:
        raise ActuatorError("pyserial not installed") from exc

    candidates = [
        port for port in list_ports.comports()
        if any(hint.lower() in (port.description or "").lower() for hint in _BRIDGE_HINTS)
    ]
    if not candidates:
        available = ", ".join(p.device for p in list_ports.comports()) or "none"
        raise ActuatorError(
            f"port: auto found no USB-UART bridge. Ports present: {available}. "
            f"Check the cable carries data (not charge-only) and that the board "
            f"is powered, then run 'python -m tools.serial_probe --list'."
        )
    if len(candidates) > 1:
        names = ", ".join(f"{p.device} ({p.description})" for p in candidates)
        raise ActuatorError(
            f"port: auto is ambiguous, {len(candidates)} bridges found: {names}. "
            f"Set actuator.port explicitly in the config."
        )

    logger.info(
        "port: auto resolved to %s (%s)", candidates[0].device, candidates[0].description
    )
    return candidates[0].device


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
        self._heartbeat_enabled = threading.Event()
        self._heartbeat_enabled.set()
        self._reader: Optional[threading.Thread] = None
        self._heartbeat: Optional[threading.Thread] = None

        self._last_status: Optional[ActuatorStatus] = None
        self._status_event = threading.Event()
        self._last_write: float = 0.0
        self._armed: bool = False
        self._errors: List[str] = []
        # True once the port has opened AND firmware has answered. Distinguishes
        # "the effector may be live and we lost control of it" (a real safety
        # event) from "we never reached the board" (a startup failure). Crying
        # wolf on the former teaches operators to ignore the log that matters.
        self._link_established: bool = False

    def connect(self) -> None:
        """Open the port, start threads, and force a known safe state.

        Raises:
            ActuatorError: If the port cannot be opened or firmware is silent.
        """
        try:
            import serial
        except ImportError as exc:
            raise ActuatorError("pyserial not installed. pip install pyserial") from exc

        self._port_name = resolve_port(self._port_name)
        try:
            self._serial = serial.Serial(
                self._port_name, self._baud, timeout=0.1, write_timeout=0.5
            )
        except (OSError, ValueError) as exc:
            raise ActuatorError(f"Could not open {self._port_name}: {exc}") from exc

        # ESP32 resets when the bridge asserts DTR; wait out the boot. Capture
        # the banner BEFORE discarding the buffer — it carries the PWM attach
        # state, which is the only report of a failure that is otherwise silent.
        time.sleep(2.0)
        try:
            pending = self._serial.read(self._serial.in_waiting or 0)
            for line in pending.decode("ascii", errors="replace").splitlines():
                if "BOOT" in line or "ATTACH FAILED" in line:
                    if "ATTACH FAILED" in line:
                        logger.critical("Firmware boot: %s", line.strip())
                    else:
                        logger.info("Firmware boot: %s", line.strip())
        except (OSError, ValueError) as exc:
            logger.debug("Could not read boot banner: %s", exc)
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
        self._link_established = True
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
            elif line.startswith("DIAG") or "ATTACH FAILED" in line:
                logger.warning("Firmware: %s", line)
            elif line.startswith("OK BOOT"):
                logger.info("Firmware: %s", line)
            elif not line.startswith("OK"):
                logger.debug("Firmware: %s", line)

    def _heartbeat_loop(self) -> None:
        """Refresh the firmware deadman when the loop is otherwise idle."""
        while self._running.is_set():
            time.sleep(HEARTBEAT_INTERVAL_S / 2.0)
            if not self._heartbeat_enabled.is_set():
                continue
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
            if not self._link_established:
                # Never reached the board, so nothing was ever energised.
                logger.debug("E-stop skipped, link was never established: %s", exc)
                return
            logger.critical(
                "E-STOP COULD NOT BE SENT: %s. Firmware deadman will cut the "
                "effector within the timeout window.", exc,
            )

    def suspend_heartbeat(self) -> None:
        """Stop refreshing the firmware deadman, WITHOUT stopping the reader.

        Diagnostic hook for verifying the deadman on real hardware. Clearing
        `_running` would stop the reader thread too, so the firmware would cut
        the effector and the host would never see the status frame proving it —
        the test would report a failure that did not happen.
        """
        self._heartbeat_enabled.clear()
        logger.warning("Heartbeat SUSPENDED — firmware deadman will fire")

    def resume_heartbeat(self) -> None:
        """Resume refreshing the deadman."""
        self._heartbeat_enabled.set()

    def send_raw(self, body: str) -> None:
        """Send an arbitrary protocol body. Diagnostics and bring-up only.

        Bypasses the typed API deliberately: 'D' and 'T' are firmware
        diagnostics that have no place in the control path.
        """
        self._write(frame(body))

    def last_status(self) -> Optional[ActuatorStatus]:
        with self._state_lock:
            return self._last_status

    def confirm_effector_off(self, timeout: float = 0.5) -> bool:
        """Wait for a status frame showing the effector de-energised."""
        if not self._link_established:
            # The board was never reached, so the effector was never armed and
            # never energised. Nothing to confirm; this is not a safety event.
            return True
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
        if self._link_established:
            logger.info("Actuator link closed")
        self._link_established = False


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
        """Arm, clearing the soft e-stop latch.

        Mirrors the firmware exactly: a CHECKSUMMED M1 sets `estopLatched =
        false` and arms. The soft latch exists so a freshly-booted or faulted
        board refuses to fire until something deliberately and verifiably arms
        it — not to make recovery require a power cycle. The barrier that needs
        human action is the physical interlock in series with the effector
        (guardrails section 2, HARD), which no software path can clear.
        """
        self.estop_latched = False
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
        """Clear the soft latch without arming. Test affordance."""
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
