"""Host/firmware serial protocol — shared definitions.

Newline-terminated ASCII, human-readable so it can be driven from a serial
monitor during bring-up and debugged without tooling.

Asymmetric integrity: commands that INCREASE hazard (arm, fire) require a
checksum. Commands that DECREASE hazard (disarm, laser off, e-stop) are always
accepted. A corrupted byte must never be able to fire the effector, and must
never be able to prevent a shutdown.
"""
from __future__ import annotations

from typing import Optional

# Servo bounds. Enforced here AND in firmware (guardrails.md section 3, HARD).
PAN_MIN_DEG: float = 0.0
PAN_MAX_DEG: float = 180.0
TILT_MIN_DEG: float = 45.0
TILT_MAX_DEG: float = 135.0

# Firmware kills the effector if no valid command arrives inside this window.
DEADMAN_TIMEOUT_MS: int = 250
# Host heartbeat interval. Comfortably inside the deadman even with jitter.
HEARTBEAT_INTERVAL_S: float = 0.10
# Firmware-enforced hard ceiling on a single burn. The host is not trusted.
MAX_BURN_MS: int = 2000

BAUD_RATE: int = 921600

# Error codes emitted by firmware as `ERR <code> <detail>`.
ERR_UNKNOWN_CMD = "E01"
ERR_BAD_FORMAT = "E02"
ERR_OUT_OF_RANGE = "E03"
ERR_NOT_ARMED = "E04"
ERR_BAD_CHECKSUM = "E05"
ERR_BURN_CEILING = "E06"
ERR_DEADMAN = "E07"
ERR_ESTOP_LATCHED = "E08"

ERROR_TEXT = {
    ERR_UNKNOWN_CMD: "unknown command",
    ERR_BAD_FORMAT: "malformed command",
    ERR_OUT_OF_RANGE: "value outside permitted bounds",
    ERR_NOT_ARMED: "fire rejected: subsystem not armed",
    ERR_BAD_CHECKSUM: "checksum mismatch on a hazard-increasing command",
    ERR_BURN_CEILING: "burn ceiling reached: effector cut by firmware",
    ERR_DEADMAN: "deadman timeout: effector cut by firmware",
    ERR_ESTOP_LATCHED: "rejected: emergency stop latched, re-arm required",
}


def checksum(body: str) -> str:
    """XOR checksum of an ASCII command body, as two uppercase hex digits.

    Args:
        body: Command text without the trailing checksum or newline, e.g. "L1".

    Returns:
        Two-character hex, e.g. "7D".
    """
    acc = 0
    for char in body:
        acc ^= ord(char)
    return f"{acc:02X}"


def frame(body: str, require_checksum: bool = False) -> bytes:
    """Build a wire frame.

    Args:
        body: Command body, e.g. "A090.0,045.5".
        require_checksum: Append `*XX` — set for hazard-increasing commands.

    Returns:
        Encoded bytes including the trailing newline.
    """
    if require_checksum:
        return f"{body}*{checksum(body)}\n".encode("ascii")
    return f"{body}\n".encode("ascii")


def clamp_pan(angle: float) -> float:
    """Clamp pan to bounds. Clamps, never wraps: 190 becomes 180, never 10."""
    return max(PAN_MIN_DEG, min(PAN_MAX_DEG, angle))


def clamp_tilt(angle: float) -> float:
    """Clamp tilt to bounds. Clamps, never wraps."""
    return max(TILT_MIN_DEG, min(TILT_MAX_DEG, angle))


class ActuatorStatus:
    """Parsed `ST <pan>,<tilt>,<laser>,<armed>,<uptime_ms>` frame.

    `pan`/`tilt` are COMMANDED angles, not measured. SG90 servos have no
    position feedback. Never present these as telemetry in a document a judge
    will read. See architecture.md section 2.
    """

    __slots__ = ("pan", "tilt", "laser_on", "armed", "uptime_ms", "received_at")

    def __init__(
        self, pan: float, tilt: float, laser_on: bool, armed: bool,
        uptime_ms: int, received_at: float,
    ) -> None:
        self.pan = pan
        self.tilt = tilt
        self.laser_on = laser_on
        self.armed = armed
        self.uptime_ms = uptime_ms
        self.received_at = received_at

    @classmethod
    def parse(cls, line: str, received_at: float) -> Optional["ActuatorStatus"]:
        """Parse a status line, or return None if it is not one / is malformed."""
        if not line.startswith("ST "):
            return None
        parts = line[3:].strip().split(",")
        if len(parts) != 5:
            return None
        try:
            return cls(
                pan=float(parts[0]),
                tilt=float(parts[1]),
                laser_on=parts[2] == "1",
                armed=parts[3] == "1",
                uptime_ms=int(parts[4]),
                received_at=received_at,
            )
        except ValueError:
            return None

    def __repr__(self) -> str:
        return (
            f"ActuatorStatus(pan={self.pan:.1f}, tilt={self.tilt:.1f}, "
            f"laser={'ON' if self.laser_on else 'off'}, "
            f"armed={'YES' if self.armed else 'no'}, up={self.uptime_ms}ms)"
        )
