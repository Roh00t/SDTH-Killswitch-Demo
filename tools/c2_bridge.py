"""C2 bridge — the Hybrid Twin. One real node, N simulated, one shared truth.

    REAL Node 1 (main.py) ──MQTT──▶ c2/node/{telemetry,event}
                                          │
                              ┌───────────▼────────────┐
                              │   FleetState (asyncio  │  node 1  = REAL
                              │   Lock — single truth) │  nodes 2+ = simulated
                              └──┬──────────┬──────────┘  threats  = simulated
              CoT UDP multicast ─┘          └─ WebSocket :8765 ─▶ dashboard
              239.2.3.1:6969                                      + MAVLink frames
                     │
                     ▼                    ┌─────────────────────────────────┐
                  WinTAK                  │ threat bearing ──▶ slew_to_cue  │
                                          │   ──▶ REAL GIMBAL PHYSICALLY    │
                                          │       SLEWS  (rate-limited 5Hz) │
                                          └─────────────────────────────────┘

Why FleetState is the only authority: a dashboard animating independently of
ATAK will visibly desync on stage. Both surfaces render from this one object.

Why the rate limiter exists: the firmware already limits slew to ~250 deg/s and
the node already gates on frame_id, but neither stops a fast CoT stream from
re-cueing the state machine dozens of times a second. Bound it at the source —
servo stutter and brownout are mechanical failures no software layer can undo.

Usage:
    python -m tools.c2_bridge
    python -m tools.c2_bridge --loopback          # venue blocks multicast
    python -m tools.c2_bridge --no-cue            # observe only, never slew
    python -m tools.c2_bridge --cot-unicast 192.168.43.1   # also send straight to a phone
    python -m tools.c2_bridge --base 1.2966,103.7764        # where the fleet sits on the map

The TAK map is OUTPUT-ONLY. Node 1's marker mirrors the real node's reported
state and reads NO LINK when it stops reporting; every other marker says
SIMULATED in its remarks. Nothing on the map is ever set by hand.
"""
from __future__ import annotations

import argparse
import asyncio
import contextlib
import ipaddress
import json
import logging
import math
import socket
import sys
import time
from dataclasses import dataclass, field, replace
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

import aiomqtt
import yaml

from helper.comms.cot import (
    COT_FRIENDLY_GROUND,
    COT_HOSTILE_AIR_UAV,
    DEMO_SITE_HAE,
    DEMO_SITE_LAT,
    DEMO_SITE_LON,
    TAK_LOOPBACK_HOST,
    TAK_LOOPBACK_PORT,
    TAK_MULTICAST_GROUP,
    TAK_MULTICAST_PORT,
    CotError,
    CotEvent,
    bearing_range,
    build_cot,
    elevation_angle,
    offset_lat_lon,
)
from helper.comms.mavlink_frames import MavlinkEncoder
from helper.comms.schemas import (
    TOPIC_NODE_EVENT,
    TOPIC_NODE_TELEMETRY,
    TOPIC_OPERATOR_AUTH,
    TOPIC_OPERATOR_TASK,
    TOPIC_SLEW_TO_CUE,
    ValidationError,
    parse_operator_task,
)

logger = logging.getLogger("c2_bridge")

ENGAGEMENT_PERIMETER_M = 350.0

# A node that has not reported for this long reads NO LINK on every surface.
# The map must never show a live-looking state for a node that went silent.
NODE_LINK_TIMEOUT_S = 10.0
# Ceiling on --stale-pad-s. Enough to absorb a TAK device whose clock runs a
# minute ahead of this laptop; short enough that a neutralised threat cannot
# linger on the map for minutes, which a 180 s window would do.
MAX_STALE_PAD_S = 60.0
# Label fields copied from node telemetry are truncated before they reach a
# CoT event, so a malformed message can never trip outbound validation.
MAX_LABEL_CHARS = 24
MAX_TARGET_ID_CHARS = 64

Dest = Tuple[str, int]

# D2: hard ceiling on cue updates toward the ESP32.
MAX_CUE_HZ = 5.0
CUE_LOWPASS_ALPHA = 0.35        # EWMA on az/el; lower = smoother, laggier
CUE_DEADBAND_DEG = 1.5          # below this, do not re-cue at all

SIM_TICK_HZ = 10.0
COT_BROADCAST_HZ = 2.0
WS_BROADCAST_HZ = 5.0
# Longest a single dashboard send may take before that client is dropped. A
# phone that auto-locks keeps its socket open but stops reading; without this,
# its full write buffer blocks the shared broadcast and freezes every screen.
WS_SEND_TIMEOUT_S = 0.5


# ---- state -----------------------------------------------------------------


@dataclass
class NodeState:
    """A defensive node. Node 1 is real; the rest are simulated."""

    node_id: str
    callsign: str
    lat: float
    lon: float
    hae: float = 20.0
    kind: str = "LASER"
    status: str = "PASSIVE SCAN"
    is_real: bool = False
    last_seen: float = field(default_factory=time.monotonic)
    target_id: Optional[str] = None
    reported: bool = False          # real node only: telemetry seen since start
    # Simulated nodes only: the threat an operator keypress tasked this asset
    # onto. A label, never an engagement. See FleetState.task_asset.
    tasked_threat: Optional[str] = None
    tasked_callsign: Optional[str] = None

    def link_state(self) -> str:
        """The state the map may claim for the real node, or NO LINK.

        NO LINK before the first report, after the node's last-will fires, and
        once reports stop for NODE_LINK_TIMEOUT_S. A stale ENGAGE on a map is
        worse than no state at all.
        """
        if not self.reported or time.monotonic() - self.last_seen > NODE_LINK_TIMEOUT_S:
            return "NO LINK"
        return self.status[:MAX_LABEL_CHARS]

    def to_cot(self) -> CotEvent:
        """Map marker. Node 1 carries its live state; every other node says
        SIMULATED, so nobody tapping a marker can mistake one for the other."""
        if self.is_real:
            state = self.link_state()
            target = (self.target_id or "-")[:MAX_TARGET_ID_CHARS] if state != "NO LINK" else "-"
            callsign = f"{self.callsign} [{state}]"
            remarks = f"REAL NODE | STATE {state} | TGT {target}"
        elif self.tasked_threat:
            callsign = f"{self.callsign} [TASKED]"
            remarks = f"SIMULATED {self.kind} | TASKED BY OPERATOR -> {self.tasked_callsign}"
        else:
            callsign = self.callsign
            remarks = f"SIMULATED {self.kind}"
        return CotEvent(
            uid=self.node_id, callsign=callsign, cot_type=COT_FRIENDLY_GROUND,
            lat=self.lat, lon=self.lon, hae=self.hae, stale_seconds=8.0,
            remarks=remarks,
        )

    def to_dict(self) -> dict:
        return {
            "node_id": self.node_id, "callsign": self.callsign, "kind": self.kind,
            "status": self.status, "is_real": self.is_real,
            "tasked_threat": self.tasked_threat,
            "lat": self.lat, "lon": self.lon,
            "stale": time.monotonic() - self.last_seen > NODE_LINK_TIMEOUT_S,
        }


@dataclass
class ThreatTrack:
    """An inbound hostile, flown along a real great-circle vector."""

    uid: str
    callsign: str
    bearing_deg: float          # bearing FROM base TO threat
    distance_m: float
    alt_m: float
    speed_mps: float
    status: str = "INBOUND"
    origin_lat: float = DEMO_SITE_LAT   # the base this threat is flying toward
    origin_lon: float = DEMO_SITE_LON
    tasked_to: Optional[str] = None     # callsign of the simulated asset tasked onto it

    @property
    def time_to_impact_s(self) -> float:
        """Drives decision-deck priority. Lower = more urgent."""
        if self.speed_mps <= 0.0 or self.status != "INBOUND":
            return float("inf")
        return max(0.0, self.distance_m / self.speed_mps)

    def position(self):
        return offset_lat_lon(
            self.origin_lat, self.origin_lon, max(0.0, self.distance_m), self.bearing_deg
        )

    def advance(self, dt: float) -> None:
        if self.status == "INBOUND":
            self.distance_m = max(0.0, self.distance_m - self.speed_mps * dt)

    def to_cot(self) -> CotEvent:
        lat, lon = self.position()
        tti_s = self.time_to_impact_s
        tti = f"{tti_s:.1f}" if math.isfinite(tti_s) else "-"
        return CotEvent(
            uid=self.uid, callsign=self.callsign, cot_type=COT_HOSTILE_AIR_UAV,
            lat=lat, lon=lon, hae=self.alt_m, speed=self.speed_mps,
            course=(self.bearing_deg + 180.0) % 360.0, stale_seconds=4.0,
            remarks=f"SIMULATED THREAT | {self.distance_m:.0f} m | TTI {tti} s"
                    + (f" | TASKED {self.tasked_to}" if self.tasked_to else ""),
        )

    def to_dict(self) -> dict:
        lat, lon = self.position()
        return {
            "uid": self.uid, "callsign": self.callsign, "status": self.status,
            "lat": lat, "lon": lon, "alt_m": self.alt_m,
            "range_m": round(self.distance_m, 1), "bearing_deg": round(self.bearing_deg, 1),
            "speed_mps": self.speed_mps, "tti_s": round(self.time_to_impact_s, 1),
            "in_perimeter": self.distance_m <= ENGAGEMENT_PERIMETER_M,
            "tasked_to": self.tasked_to,
        }


class FleetState:
    """Single source of truth for ATAK, the dashboard and the cue path.

    Thread/task-safety: every mutation and every read that must be coherent
    takes `self.lock`. The swarm simulator and the real node's MQTT telemetry
    both write here concurrently.
    """

    def __init__(
        self,
        simulated_nodes: int = 19,
        base: Tuple[float, float, float] = (DEMO_SITE_LAT, DEMO_SITE_LON, DEMO_SITE_HAE),
    ) -> None:
        """Args:
            simulated_nodes: Simulated nodes beyond the real Node 1.
            base: (lat, lon, hae) of Node 1. Everything on the map is placed
                relative to it. Set once here and never mutated.
        """
        self.base_lat, self.base_lon, self.base_hae = base
        self.lock = asyncio.Lock()
        self.nodes: Dict[str, NodeState] = {}
        self.threats: Dict[str, ThreatTrack] = {}
        self.node1_state: str = "UNKNOWN"
        self.node1_telemetry: dict = {}
        self.events: List[dict] = []

        self.nodes["KILLSWITCH_NODE_01"] = NodeState(
            node_id="KILLSWITCH_NODE_01", callsign="KILLSWITCH-01",
            lat=self.base_lat, lon=self.base_lon, hae=self.base_hae, kind="LASER",
            is_real=True,
        )
        kinds = ["INTERCEPTOR", "RF-JAMMER", "SENTRY-UGV", "LASER"]
        for i in range(2, 2 + simulated_nodes):
            lat, lon = offset_lat_lon(self.base_lat, self.base_lon, 400 + 90 * i, (i * 47) % 360)
            self.nodes[f"KILLSWITCH_NODE_{i:02d}"] = NodeState(
                node_id=f"KILLSWITCH_NODE_{i:02d}", callsign=f"KILLSWITCH-{i:02d}",
                lat=lat, lon=lon, kind=kinds[i % len(kinds)],
            )

    def seed_swarm(self, start_range_m: Optional[float] = None) -> None:
        """Populate the 4-vector swarm.

        Args:
            start_range_m: Override the initial range for every threat. On
                stage you do not want 30 seconds of dead air waiting for a
                1200 m threat to close — start them near the perimeter so the
                breach, cue and engagement happen while you are still talking.
        """
        for uid, cs, brg, dist, alt, spd in (
            ("THREAT_ALPHA_01", "SWARM-ALPHA-01", 45.0, 1200.0, 85.0, 35.0),
            ("THREAT_ALPHA_02", "SWARM-ALPHA-02", 55.0, 1350.0, 92.0, 38.0),
            ("THREAT_BETA_01", "SWARM-BETA-01", 315.0, 1000.0, 60.0, 42.0),
            ("THREAT_GAMMA_01", "SWARM-GAMMA-01", 180.0, 1500.0, 40.0, 30.0),
        ):
            self.threats[uid] = ThreatTrack(
                uid, cs, brg, start_range_m if start_range_m else dist, alt, spd,
                origin_lat=self.base_lat, origin_lon=self.base_lon,
            )

    def priority_threat(self) -> Optional[ThreatTrack]:
        """Lowest time-to-impact inside the perimeter. Caller holds the lock."""
        live = [
            t for t in self.threats.values()
            if t.status == "INBOUND" and t.distance_m <= ENGAGEMENT_PERIMETER_M
        ]
        return min(live, key=lambda t: t.time_to_impact_s) if live else None

    def task_asset(self, asset: int) -> Optional[Tuple[NodeState, ThreatTrack]]:
        """Task simulated asset `asset` onto the next unassigned threat.

        Node 1 owns the threat the cue path drives the real gimbal onto: the
        priority threat once one is inside the perimeter, otherwise the
        lowest-TTI inbound one, which is the next to breach. Each simulated
        asset takes the highest-priority threat nobody owns yet, so keys 1-3
        spread three assets over three threats instead of piling onto Node 1's.
        This only labels the asset. It never changes a threat's status, Node 1,
        or what the cue path does.

        Args:
            asset: 1..MAX_TASK_ASSETS; asset n is node KILLSWITCH_NODE_{n+1}.

        Returns:
            (asset, threat), or None with the reason logged: no such simulated
            asset, already tasked onto a live threat, or no threat left.

        Thread: the bridge's event loop, holding `self.lock`.
        """
        node = self.nodes.get(f"KILLSWITCH_NODE_{asset + 1:02d}")
        if node is None or node.is_real:
            logger.warning("Task for asset %d ignored: no such simulated asset", asset)
            return None
        inbound = sorted((t for t in self.threats.values() if t.status == "INBOUND"),
                         key=lambda t: t.time_to_impact_s)
        if node.tasked_threat and any(t.uid == node.tasked_threat for t in inbound):
            logger.info("%s is already tasked onto %s", node.callsign, node.tasked_callsign)
            return None
        node1_threat = self.priority_threat() or (inbound[0] if inbound else None)
        owned = {n.tasked_threat for n in self.nodes.values() if n.tasked_threat}
        if node1_threat is not None:
            owned.add(node1_threat.uid)
        target = next((t for t in inbound if t.uid not in owned), None)
        if target is None:
            logger.warning("Task for %s ignored: every inbound threat is already covered",
                           node.callsign)
            return None
        node.tasked_threat, node.tasked_callsign = target.uid, target.callsign
        target.tasked_to = node.callsign
        self.log("operator_tasked", f"{node.callsign} -> {target.callsign}")
        logger.info("Operator tasked %s (SIMULATED %s) onto %s",
                    node.callsign, node.kind, target.callsign)
        return node, target

    def release_task(self, threat: ThreatTrack) -> None:
        """Clear any tasking that points at `threat`. Caller holds the lock."""
        threat.tasked_to = None
        for node in self.nodes.values():
            if node.tasked_threat == threat.uid:
                node.tasked_threat = node.tasked_callsign = None

    def node1_view(self) -> Tuple[str, dict]:
        """What the dashboard may claim for the real node: (state, telemetry).

        The map marker's rule, applied to the dashboard. Before the first
        report, after the last-will, or after NODE_LINK_TIMEOUT_S of silence
        the state reads NO LINK and the telemetry is empty. Without this the
        dashboard kept a dead node's last state pill and frozen numbers on
        screen indefinitely. Caller holds the lock.
        """
        node = self.nodes.get("KILLSWITCH_NODE_01")
        if node is None or node.link_state() == "NO LINK":
            return "NO LINK", {}
        return self.node1_state, dict(self.node1_telemetry)

    def log(self, event: str, detail: str) -> None:
        self.events.append({"ts": time.time(), "event": event, "detail": detail})
        del self.events[:-60]


# ---- D2: kinematic smoothing ----------------------------------------------


class SlewRateLimiter:
    """Caps cue rate and low-passes az/el before anything reaches the ESP32.

    Three independent protections stack here, deliberately:
      1. rate cap       — at most MAX_CUE_HZ cues per second
      2. low-pass       — EWMA removes step changes that stutter the gears
      3. deadband       — below CUE_DEADBAND_DEG, do not re-cue at all
    """

    def __init__(self, max_hz: float = MAX_CUE_HZ, alpha: float = CUE_LOWPASS_ALPHA) -> None:
        self._min_interval = 1.0 / max_hz
        self._alpha = alpha
        self._last_emit = 0.0
        self._az: Optional[float] = None
        self._el: Optional[float] = None
        self._last_sent: Optional[tuple] = None

    def reset(self) -> None:
        """Clear all state, including the rate-cap timer.

        Clearing `_last_emit` matters: on a target switch the new bearing must
        be cued immediately, not held back by the cap left over from the
        previous target.
        """
        self._az = self._el = self._last_sent = None
        self._last_emit = 0.0

    @property
    def current(self):
        """Filtered ``(az, el)``, or None before the first offer.

        Exposed so the filter can be inspected independently of whether a cue
        was emitted — the filter advances on every offer, the rate cap only
        decides what reaches the wire.
        """
        return None if self._az is None else (self._az, self._el)

    def offer(self, azimuth: float, elevation: float):
        """Return a smoothed ``(az, el)`` to publish, or None to suppress."""
        now = time.monotonic()

        if self._az is None:
            self._az, self._el = azimuth, elevation
        else:
            # Shortest angular path, so 359 -> 1 does not sweep the long way.
            delta = ((azimuth - self._az + 180.0) % 360.0) - 180.0
            self._az = (self._az + self._alpha * delta) % 360.0
            self._el += self._alpha * (elevation - self._el)

        if now - self._last_emit < self._min_interval:
            return None

        if self._last_sent is not None:
            d_az = abs(((self._az - self._last_sent[0] + 180.0) % 360.0) - 180.0)
            if d_az < CUE_DEADBAND_DEG and abs(self._el - self._last_sent[1]) < CUE_DEADBAND_DEG:
                return None

        self._last_emit = now
        self._last_sent = (self._az, self._el)
        return (self._az, self._el)


# ---- transport -------------------------------------------------------------


def parse_unicast_dest(text: str) -> Dest:
    """argparse type for --cot-unicast: ``IPv4[:PORT]``, port defaulting to 6969.

    IPv4 literals only. A hostname would need DNS, which an offline venue does
    not have, and would fail at the first send rather than at launch.

    Raises:
        argparse.ArgumentTypeError: On anything that is not a usable address.
    """
    host, sep, port_text = text.partition(":")
    try:
        ip = ipaddress.IPv4Address(host)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{text!r} is not an IPv4 address: {exc}") from exc
    if ip.is_multicast:
        raise argparse.ArgumentTypeError(
            f"{ip} is a multicast group; the multicast path is on by default")
    port = TAK_MULTICAST_PORT
    if sep:
        try:
            port = int(port_text)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(f"{text!r}: port is not a number") from exc
        if not 1 <= port <= 65535:
            raise argparse.ArgumentTypeError(f"{text!r}: port must be 1-65535")
    return (str(ip), port)


def parse_base(text: str) -> Tuple[float, float, float]:
    """argparse type for --base: ``LAT,LON`` or ``LAT,LON,HAE``.

    Raises:
        argparse.ArgumentTypeError: On a malformed or out-of-range position.
    """
    parts = text.split(",")
    if len(parts) not in (2, 3):
        raise argparse.ArgumentTypeError(f"{text!r}: expected LAT,LON or LAT,LON,HAE")
    try:
        values = [float(v) for v in parts]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{text!r}: not a number") from exc
    if not all(math.isfinite(v) for v in values):
        raise argparse.ArgumentTypeError(f"{text!r}: values must be finite")
    lat, lon = values[0], values[1]
    hae = values[2] if len(values) == 3 else DEMO_SITE_HAE
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        raise argparse.ArgumentTypeError(f"{text!r}: latitude or longitude out of range")
    return (lat, lon, hae)


def parse_stale_pad(text: str) -> float:
    """argparse type for --stale-pad-s: seconds in [0, MAX_STALE_PAD_S].

    Rejected rather than clamped, so a typo is loud at launch.

    Raises:
        argparse.ArgumentTypeError: Outside the range or not a number.
    """
    try:
        value = float(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{text!r}: not a number") from exc
    if not (math.isfinite(value) and 0.0 <= value <= MAX_STALE_PAD_S):
        raise argparse.ArgumentTypeError(
            f"{text!r}: must be between 0 and {MAX_STALE_PAD_S:.0f} seconds")
    return value


def make_cot_socket(
    loopback: bool,
    interface_ip: Optional[str] = None,
    unicast: Sequence[Dest] = (),
) -> Tuple[socket.socket, List[Dest]]:
    """UDP socket for CoT, plus every destination to send each datagram to.

    Args:
        loopback: Use 127.0.0.1 unicast instead of multicast. Venue networks
            frequently block multicast — verify this path before demo day.
        interface_ip: Bind IP_MULTICAST_IF explicitly. Needed when the laptop
            has several adapters (wifi + hotspot + virtual) and the kernel
            picks the wrong one.
        unicast: Extra destinations that get a direct copy of every datagram.
            Phone hotspots and many access points drop multicast but still
            pass unicast, so this is what gets tracks onto an ATAK-CIV phone.

    Returns:
        ``(sock, dests)``: the multicast or loopback destination first, then
        the unicast destinations in the order given.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    if loopback:
        return sock, [(TAK_LOOPBACK_HOST, TAK_LOOPBACK_PORT), *unicast]
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
    if interface_ip:
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF,
                        socket.inet_aton(interface_ip))
        logger.info("Bound IP_MULTICAST_IF to %s", interface_ip)
    return sock, [(TAK_MULTICAST_GROUP, TAK_MULTICAST_PORT), *unicast]


def prepare_datagram(event: CotEvent, stale_pad_s: float = 0.0) -> bytes:
    """Serialise one event, extending its stale window by the clock-drift pad.

    The pad is clamped to [0, MAX_STALE_PAD_S] here as well as at argparse,
    so no caller can push a neutralised threat's lifetime into minutes.

    Raises:
        CotError: If the event fails outbound validation.
    """
    pad = min(max(stale_pad_s, 0.0), MAX_STALE_PAD_S)
    if pad:
        event = replace(event, stale_seconds=event.stale_seconds + pad)
    return build_cot(event)


# ---- tasks -----------------------------------------------------------------


async def task_swarm(fleet: FleetState) -> None:
    """Fly the simulated threats inward."""
    dt = 1.0 / SIM_TICK_HZ
    while True:
        async with fleet.lock:
            for t in fleet.threats.values():
                was_outside = t.distance_m > ENGAGEMENT_PERIMETER_M
                t.advance(dt)
                if was_outside and t.distance_m <= ENGAGEMENT_PERIMETER_M:
                    fleet.log("perimeter_breach",
                              f"{t.callsign} inside {ENGAGEMENT_PERIMETER_M:.0f}m")
                    logger.warning("PERIMETER BREACH %s at %.0fm", t.callsign, t.distance_m)
                if t.status == "INBOUND" and t.distance_m <= 10.0:
                    t.status = "NEUTRALIZED"
                    fleet.release_task(t)
                    fleet.log("threat_neutralized", t.callsign)
            if all(t.status != "INBOUND" for t in fleet.threats.values()):
                fleet.log("scenario_reset", "all vectors resolved")
                for t in fleet.threats.values():
                    fleet.release_task(t)
                    t.distance_m += 1200.0
                    t.status = "INBOUND"
        await asyncio.sleep(dt)


async def task_cot_broadcast(
    fleet: FleetState, sock: socket.socket, dests: Sequence[Dest], stale_pad_s: float = 0.0,
) -> None:
    """Push the whole picture to every TAK destination."""
    failing: Set[Dest] = set()
    rejected: Set[str] = set()
    while True:
        async with fleet.lock:
            events = [n.to_cot() for n in fleet.nodes.values()]
            events += [t.to_cot() for t in fleet.threats.values() if t.status == "INBOUND"]
        for ev in events:
            try:
                payload = prepare_datagram(ev, stale_pad_s)
            except CotError as exc:
                # One bad event must not take the TaskGroup, and with it the
                # whole bridge, down. Warn once per track, then skip it.
                if ev.uid not in rejected:
                    rejected.add(ev.uid)
                    logger.warning("CoT for %s failed validation, skipped: %s", ev.uid, exc)
                continue
            for dest in dests:
                try:
                    sock.sendto(payload, dest)
                    failing.discard(dest)
                except OSError as exc:
                    # DDIL: a jammed or flapping interface must not kill the
                    # bridge. The first failure per destination is a WARNING:
                    # at debug, a phone that never receives anything is silent.
                    if dest not in failing:
                        failing.add(dest)
                        logger.warning("CoT to %s:%d failing: %s (repeats logged at debug)",
                                       dest[0], dest[1], exc)
                    else:
                        logger.debug("CoT to %s:%d failed for %s: %s",
                                     dest[0], dest[1], ev.uid, exc)
        await asyncio.sleep(1.0 / COT_BROADCAST_HZ)


async def task_mqtt_consume(
    fleet: FleetState, client: aiomqtt.Client, task_token: Optional[str] = None,
) -> None:
    """Fold the REAL node's telemetry into FleetState, and apply operator tasks.

    Messages are routed by topic before anything is parsed. This loop used to
    treat every message as node telemetry, which was harmless with two node
    topics and would let an operator task overwrite Node 1's telemetry once a
    third topic was subscribed. Validate, never act on the effector: a task
    only relabels a simulated asset.

    Args:
        fleet: Shared state.
        client: Connected MQTT client.
        task_token: Shared secret operator tasks must carry. None disables the
            check, which main() warns about at launch.

    Thread: the bridge's event loop.
    """
    async with client.messages() as messages:
        await client.subscribe(TOPIC_NODE_TELEMETRY)
        await client.subscribe(TOPIC_NODE_EVENT)
        await client.subscribe(TOPIC_OPERATOR_TASK, qos=1)
        async for message in messages:
            if message.topic.matches(TOPIC_OPERATOR_TASK):
                try:
                    task = parse_operator_task(bytes(message.payload), task_token)
                except ValidationError as exc:
                    logger.warning("Rejected operator task: %s", exc)
                    continue
                async with fleet.lock:
                    fleet.task_asset(task.asset)
                continue
            if not (message.topic.matches(TOPIC_NODE_TELEMETRY)
                    or message.topic.matches(TOPIC_NODE_EVENT)):
                continue
            try:
                body = json.loads(message.payload.decode("utf-8"))
            except (ValueError, UnicodeDecodeError) as exc:
                logger.warning("Discarding malformed node message: %s", exc)
                continue
            async with fleet.lock:
                node = fleet.nodes.get("KILLSWITCH_NODE_01")
                if node is not None:
                    if body.get("event") == "node_lost":
                        # The broker publishes the node's last will when it dies.
                        # Treat it as the end of the link at once rather than as
                        # a sign of life, or the map would keep showing the dead
                        # node's final state for another NODE_LINK_TIMEOUT_S.
                        node.reported = False
                    else:
                        node.last_seen = time.monotonic()
                        node.reported = True
                        if "state" in body and "target_id" in body:
                            node.target_id = body["target_id"]
                state = body.get("state") or body.get("to")
                if state:
                    fleet.node1_state = state
                    if node is not None:
                        node.status = state
                # Telemetry carries the real gimbal angles, deadman age and
                # latency. Keep the whole dict; the dashboard picks fields.
                if "pan_deg" in body or "compute_latency_ms" in body:
                    fleet.node1_telemetry = body
                if body.get("event"):
                    fleet.log(body["event"], str(body.get("reason", ""))[:80])


async def task_cue(fleet: FleetState, client: aiomqtt.Client, enabled: bool) -> None:
    """Map the priority threat's bearing to a cue. THIS MOVES REAL SERVOS."""
    limiter = SlewRateLimiter()
    current_uid: Optional[str] = None
    while True:
        await asyncio.sleep(1.0 / (MAX_CUE_HZ * 2))
        if not enabled:
            continue
        async with fleet.lock:
            threat = fleet.priority_threat()
            if threat is None:
                if current_uid is not None:
                    limiter.reset()
                    current_uid = None
                continue
            lat, lon = threat.position()
            uid, alt = threat.uid, threat.alt_m

        az, rng = bearing_range(fleet.base_lat, fleet.base_lon, lat, lon)
        el = elevation_angle(rng, alt, fleet.base_hae)
        if uid != current_uid:
            limiter.reset()
            current_uid = uid
            logger.info("Cueing Node 1 onto %s (%.0fm, brg %.0f)", uid, rng, az)

        smoothed = limiter.offer(az, el)
        if smoothed is None:
            continue
        await client.publish(TOPIC_SLEW_TO_CUE, json.dumps({
            "azimuth": round(smoothed[0], 2),
            "elevation": round(smoothed[1], 2),
            "target_id": uid.replace("THREAT_", "TRK-")[:64],
        }), qos=1)


async def serve_client(ws: Any, clients: Set[Any]) -> None:
    """Register one dashboard connection and hold it until it closes.

    The dashboard is receive-only, and so is this socket: anything a client
    sends is read and discarded. It used to parse "authorise"/"abort" into the
    event log under a comment claiming the veto went out over MQTT. It never
    did, and on a socket reachable from the LAN that was unauthenticated input
    for no benefit. Frames are still read rather than ignored, so a chatty
    client cannot fill its receive queue and stall its own connection.

    Args:
        ws: The accepted WebSocket connection.
        clients: The shared set the broadcast sends to.

    Thread: one asyncio task per connection, on the bridge's event loop.
    """
    from websockets.exceptions import ConnectionClosed

    clients.add(ws)
    logger.info("Dashboard connected (%d total)", len(clients))
    try:
        async for _ in ws:
            pass
    except ConnectionClosed:
        pass  # a dropped phone or a closed tab is normal, not an error
    finally:
        clients.discard(ws)


def dashboard_json(payload: dict) -> str:
    """Serialise one dashboard frame as strict JSON.

    Python writes float('inf') as `Infinity` and NaN as `NaN`; JavaScript's
    JSON.parse rejects both, and the dashboard drops any frame it cannot parse.
    A neutralised threat's time-to-impact is infinite, so every frame from the
    first neutralisation to the scenario reset was dropped: the display froze
    while the header still read CONNECTED. Non-finite floats become null, which
    the dashboard already renders as a dash. This is the only place frames are
    serialised for the browser, so it covers node telemetry passed through too.

    Args:
        payload: The frame, as built by the WebSocket pump.

    Returns:
        JSON text with no non-finite numbers.

    Thread: the bridge's event loop.
    """
    def clean(value: Any) -> Any:
        if isinstance(value, float) and not math.isfinite(value):
            return None
        if isinstance(value, dict):
            return {k: clean(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [clean(v) for v in value]
        return value

    return json.dumps(clean(payload), allow_nan=False)


async def broadcast(clients: Set[Any], blob: str, timeout_s: float = WS_SEND_TIMEOUT_S) -> None:
    """Send one frame to every dashboard client, dropping any that can't keep up.

    Each client gets its own deadline. A client that times out or has closed is
    removed from `clients`, so it can't hold the others back; its keepalive
    closes it, and the dashboard's backoff reconnects it once it reads again.

    Args:
        clients: Connected dashboards. Mutated: stalled clients are removed.
        blob: The serialised frame.
        timeout_s: Per-client send deadline.

    Thread: the bridge's event loop.
    """
    from websockets.exceptions import ConnectionClosed

    async def send_one(client: Any) -> None:
        try:
            await asyncio.wait_for(client.send(blob), timeout_s)
        except (asyncio.TimeoutError, ConnectionClosed) as exc:
            if client in clients:
                clients.discard(client)
                logger.warning("Dropped a dashboard client that stopped reading (%s)",
                               type(exc).__name__)

    targets = list(clients)
    results = await asyncio.gather(*(send_one(c) for c in targets), return_exceptions=True)
    # Anything else is unexpected. Log it and drop that client rather than let
    # it cancel the TaskGroup, which would take the cue path down with it.
    for client, result in zip(targets, results):
        if isinstance(result, BaseException) and not isinstance(result, asyncio.CancelledError):
            clients.discard(client)
            logger.error("Dashboard send failed unexpectedly, client dropped: %r", result)


async def task_websocket(fleet: FleetState, host: str, port: int) -> None:
    """Serve FleetState + live MAVLink frames to the dashboard.

    Output-only: see serve_client. Binding a non-loopback host (--ws-host
    0.0.0.0) lets a phone on the same network open the dashboard.
    """
    import websockets

    clients: Set[Any] = set()
    encoders = {"KILLSWITCH_NODE_01": MavlinkEncoder(system_id=1)}

    async def pump():
        while True:
            await asyncio.sleep(1.0 / WS_BROADCAST_HZ)
            if not clients:
                continue
            async with fleet.lock:
                threats = sorted((t.to_dict() for t in fleet.threats.values()),
                                 key=lambda d: d["tti_s"])
                node1_state, node1_telemetry = fleet.node1_view()
                payload = {
                    "ts": time.time(),
                    "node1_state": node1_state,
                    "node": node1_telemetry,
                    "nodes": [n.to_dict() for n in fleet.nodes.values()],
                    "threats": threats,
                    "events": fleet.events[-20:],
                    "perimeter_m": ENGAGEMENT_PERIMETER_M,
                    "cue_enabled": True,
                }
            enc = encoders["KILLSWITCH_NODE_01"]
            payload["mavlink"] = [
                enc.heartbeat(armed=node1_state in ("ENGAGE", "OPERATOR_AUTH")).to_dict(),
                enc.global_position_int(fleet.base_lat, fleet.base_lon,
                                        fleet.base_hae).to_dict(),
            ]
            await broadcast(clients, dashboard_json(payload))

    async with websockets.serve(lambda ws: serve_client(ws, clients), host, port):
        logger.info("WebSocket serving on ws://%s:%d", host, port)
        if host not in ("localhost", "127.0.0.1", "::1"):
            from tools.multicast_test import local_ips  # stdlib-only
            for ip in local_ips():
                logger.info("Phone on this network: open http://%s:8000/c2_dashboard.html "
                            "(with http.server on 8000)", ip)
        await pump()


# ---- entry -----------------------------------------------------------------


async def run(args) -> int:
    fleet = FleetState(simulated_nodes=args.sim_nodes, base=args.base)
    fleet.seed_swarm(start_range_m=args.threat_start_m)
    sock, dests = make_cot_socket(args.loopback, args.multicast_if, args.cot_unicast or ())
    unicast_note = "".join(f" + unicast {h}:{p}" for h, p in dests[1:])
    logger.info("CoT -> %s:%d%s  |  %d nodes (1 real)  |  cue=%s",
                dests[0][0], dests[0][1], unicast_note, len(fleet.nodes),
                "ON" if not args.no_cue else "OFF")
    logger.info("Map base %.4f, %.4f  |  stale pad %.0f s",
                fleet.base_lat, fleet.base_lon, args.stale_pad_s)

    # `return` is illegal inside an `except*` block, so the exit code is carried
    # in a local and returned after the handlers.
    exit_code = 0
    try:
        async with aiomqtt.Client(hostname=args.broker, port=args.port) as client:
            logger.info("MQTT connected to %s:%d", args.broker, args.port)
            async with asyncio.TaskGroup() as tg:
                tg.create_task(task_swarm(fleet))
                tg.create_task(task_cot_broadcast(fleet, sock, dests, args.stale_pad_s))
                tg.create_task(task_mqtt_consume(fleet, client, args.task_token))
                tg.create_task(task_cue(fleet, client, not args.no_cue))
                tg.create_task(task_websocket(fleet, args.ws_host, args.ws_port))
    except* aiomqtt.MqttError as eg:
        from helper.comms.mqtt_client import broker_start_hint
        logger.critical("MQTT failure: %s. Is mosquitto running?  %s",
                        eg.exceptions[0], broker_start_hint())
        exit_code = 1
    except* asyncio.CancelledError:
        logger.info("Bridge stopped")
    finally:
        sock.close()
    return exit_code


def _run_bridge(coro) -> int:
    """Run the bridge's event loop, forcing a selector loop on Windows.

    aiomqtt drives paho through ``loop.add_reader()``. Windows has defaulted to
    ProactorEventLoop since 3.8, and Proactor does not implement add_reader, so
    the bridge dies with NotImplementedError on the first socket registration.
    The selector loop implements it.

    Python 3.14 deprecates ``asyncio.set_event_loop_policy`` and the
    ``*EventLoopPolicy`` classes (removal targeted for 3.16), so prefer
    ``asyncio.run(loop_factory=...)`` where it exists (3.12+). 3.11 has
    TaskGroup but not loop_factory, and the bridge needs TaskGroup, so 3.11 is
    the one version that still takes the policy path.

    Args:
        coro: The bridge coroutine to run to completion.

    Returns:
        The coroutine's exit code.

    Thread: called from the process main thread only.
    """
    if sys.platform == "win32":
        if sys.version_info >= (3, 12):
            return asyncio.run(coro, loop_factory=asyncio.SelectorEventLoop)
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    return asyncio.run(coro)


def load_task_token(path: str) -> Optional[str]:
    """Read `c2.auth_token` from a node config, for checking operator tasks.

    Read at launch so a wrong path fails now, not at the first keypress on
    stage. Logs where the token came from, or that tasks are unchecked.

    Raises:
        OSError: The file cannot be read.
        yaml.YAMLError: The file is not YAML.
    """
    with open(path, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}
    token = (config.get("c2") or {}).get("auth_token")
    if token:
        logger.info("Operator tasks: token checked against c2.auth_token in %s", path)
        return str(token)
    logger.warning("No c2.auth_token in %s: operator tasks will NOT be token-checked", path)
    return None


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--broker", default="localhost")
    p.add_argument("--port", type=int, default=1883)
    p.add_argument("--ws-host", default="localhost")
    p.add_argument("--ws-port", type=int, default=8765)
    p.add_argument("--sim-nodes", type=int, default=19, help="Simulated nodes beyond Node 1")
    p.add_argument("--loopback", action="store_true",
                   help="CoT to 127.0.0.1:18999 — use when the venue blocks multicast")
    p.add_argument("--multicast-if", default=None,
                   help="Bind IP_MULTICAST_IF, e.g. 192.168.1.42, when several adapters exist")
    p.add_argument("--threat-start-m", type=float, default=None,
                   help="Start all threats at this range. Use ~420 on stage so the "
                        "perimeter breach happens in seconds, not half a minute.")
    p.add_argument("--cot-unicast", action="append", type=parse_unicast_dest,
                   metavar="IP[:PORT]",
                   help="Also send every CoT datagram straight to this device. "
                        "Repeatable; port defaults to 6969. Use the ATAK phone's IP "
                        "when the network drops multicast")
    p.add_argument("--base", type=parse_base,
                   default=(DEMO_SITE_LAT, DEMO_SITE_LON, DEMO_SITE_HAE),
                   metavar="LAT,LON[,HAE]",
                   help="Where the fleet sits on the map. Default: NUS Kent Ridge")
    p.add_argument("--stale-pad-s", type=parse_stale_pad, default=0.0, metavar="SECONDS",
                   help=f"Add to every track's stale time (0-{MAX_STALE_PAD_S:.0f}). Use "
                        "when the TAK device's clock runs ahead and tracks vanish on arrival")
    p.add_argument("--no-cue", action="store_true",
                   help="Do not publish slew_to_cue — observe only, servos stay put")
    p.add_argument("--config", default="config/bench.yaml",
                   help="Node config. Only c2.auth_token is read, to check operator "
                        "tasks. Pass the same file as main.py and the console")
    args = p.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
                        datefmt="%H:%M:%S")
    try:
        args.task_token = load_task_token(args.config)
    except (OSError, yaml.YAMLError) as exc:
        logger.critical("Cannot read --config %s: %s", args.config, exc)
        return 2
    try:
        return _run_bridge(run(args))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
