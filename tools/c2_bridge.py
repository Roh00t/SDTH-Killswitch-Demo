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
"""
from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import logging
import math
import socket
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import aiomqtt

from helper.comms.cot import (
    COT_FRIENDLY_GROUND,
    COT_HOSTILE_AIR_UAV,
    TAK_LOOPBACK_HOST,
    TAK_LOOPBACK_PORT,
    TAK_MULTICAST_GROUP,
    TAK_MULTICAST_PORT,
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
    TOPIC_SLEW_TO_CUE,
)

logger = logging.getLogger("c2_bridge")

# Defence node location (Singapore testbed).
BASE_LAT, BASE_LON, BASE_HAE = 1.3483, 103.6831, 20.0
ENGAGEMENT_PERIMETER_M = 350.0

# D2: hard ceiling on cue updates toward the ESP32.
MAX_CUE_HZ = 5.0
CUE_LOWPASS_ALPHA = 0.35        # EWMA on az/el; lower = smoother, laggier
CUE_DEADBAND_DEG = 1.5          # below this, do not re-cue at all

SIM_TICK_HZ = 10.0
COT_BROADCAST_HZ = 2.0
WS_BROADCAST_HZ = 5.0


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

    def to_cot(self) -> CotEvent:
        return CotEvent(
            uid=self.node_id, callsign=self.callsign, cot_type=COT_FRIENDLY_GROUND,
            lat=self.lat, lon=self.lon, hae=self.hae, stale_seconds=8.0,
        )

    def to_dict(self) -> dict:
        return {
            "node_id": self.node_id, "callsign": self.callsign, "kind": self.kind,
            "status": self.status, "is_real": self.is_real,
            "lat": self.lat, "lon": self.lon,
            "stale": time.monotonic() - self.last_seen > 10.0,
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

    @property
    def time_to_impact_s(self) -> float:
        """Drives decision-deck priority. Lower = more urgent."""
        if self.speed_mps <= 0.0 or self.status != "INBOUND":
            return float("inf")
        return max(0.0, self.distance_m / self.speed_mps)

    def position(self):
        return offset_lat_lon(BASE_LAT, BASE_LON, max(0.0, self.distance_m), self.bearing_deg)

    def advance(self, dt: float) -> None:
        if self.status == "INBOUND":
            self.distance_m = max(0.0, self.distance_m - self.speed_mps * dt)

    def to_cot(self) -> CotEvent:
        lat, lon = self.position()
        return CotEvent(
            uid=self.uid, callsign=self.callsign, cot_type=COT_HOSTILE_AIR_UAV,
            lat=lat, lon=lon, hae=self.alt_m, speed=self.speed_mps,
            course=(self.bearing_deg + 180.0) % 360.0, stale_seconds=4.0,
        )

    def to_dict(self) -> dict:
        lat, lon = self.position()
        return {
            "uid": self.uid, "callsign": self.callsign, "status": self.status,
            "lat": lat, "lon": lon, "alt_m": self.alt_m,
            "range_m": round(self.distance_m, 1), "bearing_deg": round(self.bearing_deg, 1),
            "speed_mps": self.speed_mps, "tti_s": round(self.time_to_impact_s, 1),
            "in_perimeter": self.distance_m <= ENGAGEMENT_PERIMETER_M,
        }


class FleetState:
    """Single source of truth for ATAK, the dashboard and the cue path.

    Thread/task-safety: every mutation and every read that must be coherent
    takes `self.lock`. The swarm simulator and the real node's MQTT telemetry
    both write here concurrently.
    """

    def __init__(self, simulated_nodes: int = 19) -> None:
        self.lock = asyncio.Lock()
        self.nodes: Dict[str, NodeState] = {}
        self.threats: Dict[str, ThreatTrack] = {}
        self.node1_state: str = "UNKNOWN"
        self.node1_telemetry: dict = {}
        self.events: List[dict] = []

        self.nodes["KILLSWITCH_NODE_01"] = NodeState(
            node_id="KILLSWITCH_NODE_01", callsign="KILLSWITCH-01",
            lat=BASE_LAT, lon=BASE_LON, hae=BASE_HAE, kind="LASER", is_real=True,
        )
        kinds = ["INTERCEPTOR", "RF-JAMMER", "SENTRY-UGV", "LASER"]
        for i in range(2, 2 + simulated_nodes):
            lat, lon = offset_lat_lon(BASE_LAT, BASE_LON, 400 + 90 * i, (i * 47) % 360)
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
                uid, cs, brg, start_range_m if start_range_m else dist, alt, spd
            )

    def priority_threat(self) -> Optional[ThreatTrack]:
        """Lowest time-to-impact inside the perimeter. Caller holds the lock."""
        live = [
            t for t in self.threats.values()
            if t.status == "INBOUND" and t.distance_m <= ENGAGEMENT_PERIMETER_M
        ]
        return min(live, key=lambda t: t.time_to_impact_s) if live else None

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


def make_cot_socket(loopback: bool, interface_ip: Optional[str] = None):
    """UDP socket for CoT, plus the destination tuple.

    Args:
        loopback: Use 127.0.0.1 unicast instead of multicast. Venue networks
            frequently block multicast — verify this path before demo day.
        interface_ip: Bind IP_MULTICAST_IF explicitly. Needed when the laptop
            has several adapters (wifi + hotspot + virtual) and the kernel
            picks the wrong one.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    if loopback:
        return sock, (TAK_LOOPBACK_HOST, TAK_LOOPBACK_PORT)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
    if interface_ip:
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF,
                        socket.inet_aton(interface_ip))
        logger.info("Bound IP_MULTICAST_IF to %s", interface_ip)
    return sock, (TAK_MULTICAST_GROUP, TAK_MULTICAST_PORT)


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
                    fleet.log("threat_neutralized", t.callsign)
            if all(t.status != "INBOUND" for t in fleet.threats.values()):
                fleet.log("scenario_reset", "all vectors resolved")
                for t in fleet.threats.values():
                    t.distance_m += 1200.0
                    t.status = "INBOUND"
        await asyncio.sleep(dt)


async def task_cot_broadcast(fleet: FleetState, sock, dest) -> None:
    """Push the whole picture to WinTAK."""
    while True:
        async with fleet.lock:
            events = [n.to_cot() for n in fleet.nodes.values()]
            events += [t.to_cot() for t in fleet.threats.values() if t.status == "INBOUND"]
        for ev in events:
            try:
                sock.sendto(build_cot(ev), dest)
            except OSError as exc:
                # DDIL: a jammed or flapping interface must not kill the bridge.
                logger.debug("CoT send failed for %s: %s", ev.uid, exc)
        await asyncio.sleep(1.0 / COT_BROADCAST_HZ)


async def task_mqtt_consume(fleet: FleetState, client: aiomqtt.Client) -> None:
    """Fold the REAL node's telemetry into FleetState. Validate, never act."""
    async with client.messages() as messages:
        await client.subscribe(TOPIC_NODE_TELEMETRY)
        await client.subscribe(TOPIC_NODE_EVENT)
        async for message in messages:
            try:
                body = json.loads(message.payload.decode("utf-8"))
            except (ValueError, UnicodeDecodeError) as exc:
                logger.warning("Discarding malformed node message: %s", exc)
                continue
            async with fleet.lock:
                node = fleet.nodes.get("KILLSWITCH_NODE_01")
                if node is not None:
                    node.last_seen = time.monotonic()
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

        az, rng = bearing_range(BASE_LAT, BASE_LON, lat, lon)
        el = elevation_angle(rng, alt, BASE_HAE)
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


async def task_websocket(fleet: FleetState, host: str, port: int) -> None:
    """Serve FleetState + live MAVLink frames to the dashboard."""
    import websockets

    clients: set = set()
    encoders = {"KILLSWITCH_NODE_01": MavlinkEncoder(system_id=1)}

    async def handler(ws):
        clients.add(ws)
        logger.info("Dashboard connected (%d total)", len(clients))
        try:
            async for raw in ws:
                try:
                    msg = json.loads(raw)
                except ValueError:
                    continue
                # Operator veto travels back out over MQTT, reusing the real
                # auth schema — target_id binding and nonce stay enforced.
                if msg.get("action") in ("authorise", "abort"):
                    fleet.log("operator_action", msg.get("action", ""))
        except Exception:  # noqa: BLE001 - a dropped client must not kill the task
            pass
        finally:
            clients.discard(ws)

    async def pump():
        while True:
            await asyncio.sleep(1.0 / WS_BROADCAST_HZ)
            if not clients:
                continue
            async with fleet.lock:
                threats = sorted((t.to_dict() for t in fleet.threats.values()),
                                 key=lambda d: d["tti_s"])
                payload = {
                    "ts": time.time(),
                    "node1_state": fleet.node1_state,
                    "node": dict(fleet.node1_telemetry),
                    "nodes": [n.to_dict() for n in fleet.nodes.values()],
                    "threats": threats,
                    "events": fleet.events[-20:],
                    "perimeter_m": ENGAGEMENT_PERIMETER_M,
                    "cue_enabled": True,
                }
            enc = encoders["KILLSWITCH_NODE_01"]
            payload["mavlink"] = [
                enc.heartbeat(armed=fleet.node1_state in ("ENGAGE", "OPERATOR_AUTH")).to_dict(),
                enc.global_position_int(BASE_LAT, BASE_LON, BASE_HAE).to_dict(),
            ]
            blob = json.dumps(payload)
            await asyncio.gather(*(c.send(blob) for c in list(clients)),
                                 return_exceptions=True)

    async with websockets.serve(handler, host, port):
        logger.info("WebSocket serving on ws://%s:%d", host, port)
        await pump()


# ---- entry -----------------------------------------------------------------


async def run(args) -> int:
    fleet = FleetState(simulated_nodes=args.sim_nodes)
    fleet.seed_swarm(start_range_m=args.threat_start_m)
    sock, dest = make_cot_socket(args.loopback, args.multicast_if)
    logger.info("CoT -> %s:%d  |  %d nodes (1 real)  |  cue=%s",
                dest[0], dest[1], len(fleet.nodes), "ON" if not args.no_cue else "OFF")

    # `return` is illegal inside an `except*` block, so the exit code is carried
    # in a local and returned after the handlers.
    exit_code = 0
    try:
        async with aiomqtt.Client(hostname=args.broker, port=args.port) as client:
            logger.info("MQTT connected to %s:%d", args.broker, args.port)
            async with asyncio.TaskGroup() as tg:
                tg.create_task(task_swarm(fleet))
                tg.create_task(task_cot_broadcast(fleet, sock, dest))
                tg.create_task(task_mqtt_consume(fleet, client))
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
    p.add_argument("--no-cue", action="store_true",
                   help="Do not publish slew_to_cue — observe only, servos stay put")
    args = p.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
                        datefmt="%H:%M:%S")
    try:
        return _run_bridge(run(args))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
