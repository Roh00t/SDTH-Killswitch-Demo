"""Cursor on Target (CoT) — the ATAK/WinTAK interoperability layer.

CoT is the wire format TAK products speak. Emitting it is what makes the claim
"interoperable with existing military C2" demonstrable rather than aspirational:
a judge can open WinTAK on their own laptop and watch our tracks populate.

This module is BIDIRECTIONAL and that is the point:

    outbound  FleetState -> CoT XML -> UDP multicast -> WinTAK map
    inbound   CoT XML    -> bearing/range -> c2/radar/slew_to_cue -> REAL GIMBAL

The inbound path is what turns a map icon into physical motion. It reuses the
already-validated `slew_to_cue` schema and `cue_to_gimbal()`, so a threat track
moving on the map drives the servos through machinery that is already tested.

Threat model: inbound CoT arrives over UDP multicast from anything on the LAN.
Treat every payload as hostile until parsed and range-checked, exactly as
`helper/comms/schemas.py` does for MQTT (guardrails.md section 3).
"""
from __future__ import annotations

import logging
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from typing import Optional, Tuple

logger = logging.getLogger(__name__)

# ---- network ---------------------------------------------------------------

TAK_MULTICAST_GROUP: str = "239.2.3.1"
TAK_MULTICAST_PORT: int = 6969
# Loopback fallback for venues that block multicast. Test BOTH before demo day.
TAK_LOOPBACK_HOST: str = "127.0.0.1"
TAK_LOOPBACK_PORT: int = 18999

# ---- MIL-STD-2525 / CoT type atoms -----------------------------------------
# CoT type grammar: a-<affiliation>-<battle dimension>-...
#   affiliation: f=friendly  h=hostile  u=unknown  n=neutral
#   dimension:   G=ground    A=air      S=sea surface
COT_FRIENDLY_GROUND: str = "a-f-G"          # our laser node
COT_FRIENDLY_GROUND_UNIT: str = "a-f-G-U-C"  # friendly ground combat unit
COT_HOSTILE_AIR_UAV: str = "a-h-A-M-F-Q"    # hostile military UAV
COT_UNKNOWN_AIR: str = "a-u-A"              # unidentified air track
COT_NEUTRAL_AIR: str = "a-n-A"

# Colours the dashboard mirrors so ATAK and the web UI agree (plan: D-patch 4).
AFFILIATION_COLOUR = {
    "f": "#3b82f6",  # friendly blue
    "h": "#ef4444",  # hostile red
    "u": "#eab308",  # unknown yellow
    "n": "#22c55e",  # neutral green
}

EARTH_RADIUS_M: float = 6371000.0

# Inbound hardening. A CoT event is a few hundred bytes; anything larger is
# either malformed or an attempt to exhaust the parser.
MAX_COT_BYTES: int = 8192
UID_PATTERN = re.compile(r"^[A-Za-z0-9_.\-]{1,64}$")


class CotError(ValueError):
    """CoT payload rejected. Message is safe to log."""


@dataclass(frozen=True)
class CotEvent:
    """A parsed or outbound CoT track.

    Attributes:
        uid: Stable track identifier. Charset-restricted — these strings reach
            log lines and operator displays.
        callsign: Human-readable label shown on the ATAK map.
        cot_type: CoT type atom, e.g. ``a-h-A-M-F-Q``.
        lat: Latitude, degrees, -90..90.
        lon: Longitude, degrees, -180..180.
        hae: Height above ellipsoid, metres.
        speed: Ground speed, m/s.
        course: Course over ground, degrees true.
        stale_seconds: How long the track remains valid on the map. Tracks that
            stop refreshing drop off automatically, which is the behaviour we
            want for a neutralised threat.
    """

    uid: str
    callsign: str
    cot_type: str
    lat: float
    lon: float
    hae: float = 0.0
    speed: float = 0.0
    course: float = 0.0
    stale_seconds: float = 5.0

    @property
    def affiliation(self) -> str:
        """'f', 'h', 'u' or 'n' — the second atom of the CoT type."""
        parts = self.cot_type.split("-")
        return parts[1] if len(parts) > 1 else "u"

    @property
    def is_hostile(self) -> bool:
        return self.affiliation == "h"

    @property
    def colour(self) -> str:
        """Hex colour the dashboard uses, so both surfaces agree."""
        return AFFILIATION_COLOUR.get(self.affiliation, "#94a3b8")


# ---- outbound --------------------------------------------------------------


def build_cot(event: CotEvent, now: Optional[datetime] = None) -> bytes:
    """Serialise a CoT event to wire XML.

    Args:
        event: The track to publish.
        now: Override for the timestamp, for deterministic tests.

    Returns:
        UTF-8 encoded CoT XML, ready for ``sendto``.

    Raises:
        CotError: If the event fails range or charset validation. We validate
            OUTBOUND too — a malformed track we emit corrupts someone else's
            common operating picture, which is worse than dropping it.
    """
    _validate_uid(event.uid)
    _validate_latlon(event.lat, event.lon)
    if not event.cot_type or not event.cot_type.startswith("a-"):
        raise CotError(f"cot_type {event.cot_type!r} is not a CoT atom")

    stamp = now or datetime.now(timezone.utc)
    time_s = _iso(stamp)
    stale_s = _iso(stamp + timedelta(seconds=max(1.0, event.stale_seconds)))

    # Built by hand rather than via ElementTree: TAK parsers are order- and
    # whitespace-sensitive in practice, and this keeps the wire form obvious
    # when debugging with tcpdump.
    return (
        f'<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        f'<event version="2.0" uid="{_esc(event.uid)}" type="{_esc(event.cot_type)}" '
        f'time="{time_s}" start="{time_s}" stale="{stale_s}" how="m-g">'
        f'<point lat="{event.lat:.7f}" lon="{event.lon:.7f}" hae="{event.hae:.1f}" '
        f'ce="9.9" le="9.9"/>'
        f"<detail>"
        f'<contact callsign="{_esc(event.callsign)}"/>'
        f'<track speed="{event.speed:.2f}" course="{event.course:.2f}"/>'
        f'<remarks>KILLSWITCH SDDE</remarks>'
        f"</detail>"
        f"</event>"
    ).encode("utf-8")


# ---- inbound ---------------------------------------------------------------


def parse_cot(payload: bytes) -> CotEvent:
    """Parse inbound CoT XML into a validated event.

    Args:
        payload: Raw UDP datagram.

    Returns:
        A validated CotEvent.

    Raises:
        CotError: On oversize, malformed XML, missing elements, or out-of-range
            values. Never partially applies — the caller logs and discards.
    """
    if len(payload) > MAX_COT_BYTES:
        raise CotError(f"payload {len(payload)}B exceeds {MAX_COT_BYTES}B cap")

    try:
        text = payload.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise CotError(f"not valid UTF-8: {exc}") from exc

    # ElementTree does not resolve external entities, and the size cap above
    # bounds entity-expansion attacks. Do not swap this for a parser with DTD
    # support without revisiting that.
    try:
        root = ET.fromstring(text)
    except ET.ParseError as exc:
        raise CotError(f"malformed XML: {exc}") from exc

    if root.tag != "event":
        raise CotError(f"root element is {root.tag!r}, expected 'event'")

    point = root.find("point")
    if point is None:
        raise CotError("missing <point>")

    uid = root.get("uid", "")
    _validate_uid(uid)

    lat = _float_attr(point, "lat")
    lon = _float_attr(point, "lon")
    _validate_latlon(lat, lon)

    detail = root.find("detail")
    callsign = uid
    speed = course = 0.0
    if detail is not None:
        contact = detail.find("contact")
        if contact is not None:
            callsign = contact.get("callsign", uid)[:64]
        track = detail.find("track")
        if track is not None:
            speed = _float_attr(track, "speed", default=0.0)
            course = _float_attr(track, "course", default=0.0)

    return CotEvent(
        uid=uid,
        callsign=callsign,
        cot_type=root.get("type", COT_UNKNOWN_AIR),
        lat=lat,
        lon=lon,
        hae=_float_attr(point, "hae", default=0.0),
        speed=speed,
        course=course,
    )


# ---- geodesy ---------------------------------------------------------------


def bearing_range(
    from_lat: float, from_lon: float, to_lat: float, to_lon: float
) -> Tuple[float, float]:
    """Initial bearing and great-circle range between two points.

    This is the bridge between the map and the gimbal: the bearing feeds
    ``slew_to_cue.azimuth`` and the range gates whether the threat is inside the
    engagement perimeter.

    Args:
        from_lat: Observer latitude (our node).
        from_lon: Observer longitude.
        to_lat: Target latitude.
        to_lon: Target longitude.

    Returns:
        ``(bearing_deg, range_m)`` with bearing in [0, 360).
    """
    phi1, phi2 = math.radians(from_lat), math.radians(to_lat)
    dlon = math.radians(to_lon - from_lon)

    y = math.sin(dlon) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlon)
    bearing = (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

    dphi = phi2 - phi1
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlon / 2) ** 2
    rng = 2 * EARTH_RADIUS_M * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return (bearing, rng)


def offset_lat_lon(
    lat: float, lon: float, distance_m: float, bearing_deg: float
) -> Tuple[float, float]:
    """Destination point given a start, a range and a bearing.

    Used by the swarm simulator to fly threats along real great-circle vectors
    rather than naive lat/lon deltas, which visibly skew away from the equator.

    Args:
        lat: Start latitude.
        lon: Start longitude.
        distance_m: Range along the bearing.
        bearing_deg: Initial bearing, degrees true.

    Returns:
        ``(lat, lon)`` of the destination.
    """
    ang = distance_m / EARTH_RADIUS_M
    brg = math.radians(bearing_deg)
    phi1, lam1 = math.radians(lat), math.radians(lon)

    phi2 = math.asin(
        math.sin(phi1) * math.cos(ang) + math.cos(phi1) * math.sin(ang) * math.cos(brg)
    )
    lam2 = lam1 + math.atan2(
        math.sin(brg) * math.sin(ang) * math.cos(phi1),
        math.cos(ang) - math.sin(phi1) * math.sin(phi2),
    )
    return (math.degrees(phi2), (math.degrees(lam2) + 540.0) % 360.0 - 180.0)


def elevation_angle(range_m: float, altitude_m: float, observer_hae_m: float = 0.0) -> float:
    """Elevation angle to a target, degrees above horizontal.

    Feeds ``slew_to_cue.elevation``. Clamped to the schema's [-90, 90].
    """
    if range_m <= 0.0:
        return 0.0
    return max(-90.0, min(90.0, math.degrees(math.atan2(altitude_m - observer_hae_m, range_m))))


# ---- internals -------------------------------------------------------------


def _iso(stamp: datetime) -> str:
    return stamp.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


def _esc(text: str) -> str:
    """Escape XML attribute content. Callsigns are operator-supplied."""
    return (
        str(text)
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def _validate_uid(uid: str) -> None:
    if not isinstance(uid, str) or not UID_PATTERN.match(uid):
        raise CotError(f"uid {uid!r} must be 1-64 chars of [A-Za-z0-9_.-]")


def _validate_latlon(lat: float, lon: float) -> None:
    if not math.isfinite(lat) or not -90.0 <= lat <= 90.0:
        raise CotError(f"lat {lat} outside [-90, 90] or non-finite")
    if not math.isfinite(lon) or not -180.0 <= lon <= 180.0:
        raise CotError(f"lon {lon} outside [-180, 180] or non-finite")


def _float_attr(node: ET.Element, name: str, default: Optional[float] = None) -> float:
    raw = node.get(name)
    if raw is None:
        if default is None:
            raise CotError(f"missing attribute {name!r}")
        return default
    try:
        value = float(raw)
    except (TypeError, ValueError) as exc:
        raise CotError(f"attribute {name}={raw!r} is not a number") from exc
    if not math.isfinite(value):
        # NaN/Infinity are representable in XML text and must never reach a
        # PWM calculation.
        raise CotError(f"attribute {name}={raw!r} is not finite")
    return value
