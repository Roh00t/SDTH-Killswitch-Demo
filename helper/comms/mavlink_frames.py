"""Genuine MAVLink v2 wire frames via pymavlink.

Deliberately NOT string formatting. The claim "built on the same protocols every
MAVLink drone already speaks" is only worth making if a judge can decode the
bytes on the spot — so we generate real frames with `pymavlink` and ship the
hex/base64 to the dashboard's protocol panel.

Verify on stage:
    >>> from helper.comms.mavlink_frames import MavlinkEncoder, decode_hex
    >>> enc = MavlinkEncoder(); frame = enc.heartbeat()
    >>> decode_hex(frame.hex_str).get_type()
    'HEARTBEAT'

Frames here are for TELEMETRY DISPLAY and interoperability demonstration. They
are not in the effector command path — that remains the serial protocol with
its asymmetric checksums and firmware deadman (guardrails.md section 2).
"""
from __future__ import annotations

import base64
import logging
import time
from dataclasses import dataclass
from typing import Optional

from pymavlink.dialects.v20 import common as mavlink2

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class MavlinkFrame:
    """One encoded MAVLink message plus its transport encodings."""

    name: str
    raw: bytes
    hex_str: str
    b64: str
    summary: str

    @property
    def length(self) -> int:
        return len(self.raw)

    def to_dict(self) -> dict:
        """Shape the dashboard protocol panel consumes."""
        return {
            "proto": "MAVLINK2",
            "msg": self.name,
            "len": self.length,
            "hex": self.hex_str,
            "b64": self.b64,
            "summary": self.summary,
        }


class _ByteSink:
    """Minimal file-like sink so MAVLink writes into memory, not a socket."""

    def __init__(self) -> None:
        self.buf = bytearray()

    def write(self, data: bytes) -> None:
        self.buf.extend(data)

    def take(self) -> bytes:
        out = bytes(self.buf)
        self.buf.clear()
        return out


class MavlinkEncoder:
    """Builds real MAVLink v2 frames for a given system/component id.

    Thread-safety: not thread-safe (sequence numbers increment per frame).
    Own one encoder per logical vehicle/node and confine it to one task.
    """

    def __init__(self, system_id: int = 1, component_id: int = 1) -> None:
        """Args:
            system_id: MAVLink system id. Use the node number so distinct nodes
                appear as distinct systems on the wire.
            component_id: MAVLink component id.
        """
        self._sink = _ByteSink()
        self._mav = mavlink2.MAVLink(
            self._sink, srcSystem=system_id, srcComponent=component_id
        )
        self._mav.robust_parsing = True
        self._boot = time.monotonic()
        self.system_id = system_id

    def _emit(self, name: str, summary: str) -> MavlinkFrame:
        raw = self._sink.take()
        return MavlinkFrame(
            name=name,
            raw=raw,
            hex_str=raw.hex(),
            b64=base64.b64encode(raw).decode("ascii"),
            summary=summary,
        )

    def heartbeat(self, armed: bool = False) -> MavlinkFrame:
        """HEARTBEAT — the liveness frame every MAVLink system emits at ~1 Hz."""
        base_mode = (
            mavlink2.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0
        ) | mavlink2.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
        self._mav.heartbeat_send(
            type=mavlink2.MAV_TYPE_ONBOARD_CONTROLLER,
            autopilot=mavlink2.MAV_AUTOPILOT_INVALID,
            base_mode=base_mode,
            custom_mode=0,
            system_status=(
                mavlink2.MAV_STATE_ACTIVE if armed else mavlink2.MAV_STATE_STANDBY
            ),
        )
        return self._emit(
            "HEARTBEAT",
            f"sys={self.system_id} type=ONBOARD_CONTROLLER "
            f"state={'ACTIVE' if armed else 'STANDBY'}",
        )

    def sys_status(self, battery_pct: int = 100, load_pct: float = 0.0) -> MavlinkFrame:
        """SYS_STATUS — load, power and comms health."""
        sensors = (
            mavlink2.MAV_SYS_STATUS_SENSOR_3D_GYRO
            | mavlink2.MAV_SYS_STATUS_SENSOR_GPS
        )
        self._mav.sys_status_send(
            onboard_control_sensors_present=sensors,
            onboard_control_sensors_enabled=sensors,
            onboard_control_sensors_health=sensors,
            load=int(max(0.0, min(100.0, load_pct)) * 10),  # 0.1% units
            voltage_battery=12000,                          # mV
            current_battery=-1,                             # unknown
            battery_remaining=int(max(0, min(100, battery_pct))),
            drop_rate_comm=0,
            errors_comm=0,
            errors_count1=0,
            errors_count2=0,
            errors_count3=0,
            errors_count4=0,
        )
        return self._emit(
            "SYS_STATUS", f"batt={battery_pct}% load={load_pct:.1f}% comm_drop=0"
        )

    def global_position_int(
        self,
        lat: float,
        lon: float,
        alt_m: float,
        heading_deg: float = 0.0,
        vx_ms: float = 0.0,
        vy_ms: float = 0.0,
        vz_ms: float = 0.0,
    ) -> MavlinkFrame:
        """GLOBAL_POSITION_INT — fused position, the standard track frame.

        Note the fixed-point units MAVLink mandates: degrees are degE7 integers
        and altitudes are millimetres. Getting these wrong is the single most
        common MAVLink integration bug, and it puts your track in the sea.
        """
        self._mav.global_position_int_send(
            time_boot_ms=int((time.monotonic() - self._boot) * 1000) & 0xFFFFFFFF,
            lat=int(lat * 1e7),
            lon=int(lon * 1e7),
            alt=int(alt_m * 1000),
            relative_alt=int(alt_m * 1000),
            vx=int(vx_ms * 100),
            vy=int(vy_ms * 100),
            vz=int(vz_ms * 100),
            hdg=int((heading_deg % 360.0) * 100),
        )
        return self._emit(
            "GLOBAL_POSITION_INT",
            f"lat={lat:.6f} lon={lon:.6f} alt={alt_m:.0f}m hdg={heading_deg:.0f}deg",
        )


def decode_hex(hex_str: str) -> Optional[object]:
    """Decode a hex frame back to a MAVLink message. Proof the bytes are real.

    Args:
        hex_str: Hex string from `MavlinkFrame.hex_str`.

    Returns:
        The decoded message, or None if it does not parse.
    """
    try:
        mav = mavlink2.MAVLink(_ByteSink())
        mav.robust_parsing = True
        msgs = mav.parse_buffer(bytes.fromhex(hex_str))
        return msgs[0] if msgs else None
    except (ValueError, AttributeError) as exc:
        logger.debug("MAVLink decode failed: %s", exc)
        return None
