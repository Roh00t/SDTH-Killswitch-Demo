"""CoT serialisation, hostile-input rejection, geodesy, and cue rate limiting.

No broker, no network, no WinTAK.
"""
from __future__ import annotations

import math

import pytest

from helper.comms.cot import (
    COT_FRIENDLY_GROUND,
    COT_HOSTILE_AIR_UAV,
    MAX_COT_BYTES,
    CotError,
    CotEvent,
    bearing_range,
    build_cot,
    elevation_angle,
    offset_lat_lon,
    parse_cot,
)

BASE_LAT, BASE_LON = 1.3483, 103.6831


def hostile(**kw) -> CotEvent:
    body = dict(uid="THREAT_ALPHA_01", callsign="SWARM-ALPHA-01",
                cot_type=COT_HOSTILE_AIR_UAV, lat=1.3560, lon=103.6905,
                hae=85.0, speed=35.0, course=225.0)
    body.update(kw)
    return CotEvent(**body)


class TestRoundTrip:
    def test_hostile_track_round_trips(self):
        original = hostile()
        parsed = parse_cot(build_cot(original))
        assert parsed.uid == original.uid
        assert parsed.callsign == original.callsign
        assert parsed.cot_type == COT_HOSTILE_AIR_UAV
        assert parsed.lat == pytest.approx(original.lat, abs=1e-6)
        assert parsed.lon == pytest.approx(original.lon, abs=1e-6)
        assert parsed.speed == pytest.approx(35.0)

    def test_affiliation_and_colour_derive_from_type(self):
        assert hostile().is_hostile is True
        assert hostile().colour == "#ef4444"
        friendly = hostile(cot_type=COT_FRIENDLY_GROUND, uid="NODE_01")
        assert friendly.is_hostile is False
        assert friendly.colour == "#3b82f6"

    def test_xml_is_wellformed_and_declares_encoding(self):
        xml = build_cot(hostile())
        assert xml.startswith(b'<?xml version="1.0" encoding="UTF-8"')
        assert b"<point" in xml and b"<detail>" in xml

    def test_callsign_is_xml_escaped(self):
        """Callsigns are operator-supplied and must not break the document."""
        parsed = parse_cot(build_cot(hostile(callsign='EVIL"><script>')))
        assert parsed.callsign == 'EVIL"><script>'


class TestHostileInput:
    def test_oversize_rejected_before_parsing(self):
        with pytest.raises(CotError, match="exceeds"):
            parse_cot(b"x" * (MAX_COT_BYTES + 1))

    def test_malformed_xml_rejected(self):
        with pytest.raises(CotError, match="malformed XML"):
            parse_cot(b"<event><<<")

    def test_wrong_root_element_rejected(self):
        with pytest.raises(CotError, match="expected 'event'"):
            parse_cot(b'<?xml version="1.0"?><notevent/>')

    def test_missing_point_rejected(self):
        with pytest.raises(CotError, match="missing <point>"):
            parse_cot(b'<?xml version="1.0"?><event uid="A" type="a-h-A"/>')

    @pytest.mark.parametrize("bad_uid", ["", "../../etc/passwd", "a" * 65, "drop table;"])
    def test_hostile_uid_rejected(self, bad_uid):
        xml = build_cot(hostile()).replace(b"THREAT_ALPHA_01", bad_uid.encode() or b"")
        with pytest.raises(CotError):
            parse_cot(xml)

    @pytest.mark.parametrize("literal", [b"NaN", b"Infinity", b"-Infinity"])
    def test_non_finite_coordinates_rejected(self, literal):
        """NaN must never reach a PWM calculation via the cue path."""
        xml = build_cot(hostile()).replace(b'lat="1.3560000"', b'lat="' + literal + b'"')
        with pytest.raises(CotError, match="finite"):
            parse_cot(xml)

    def test_out_of_range_latitude_rejected(self):
        xml = build_cot(hostile()).replace(b'lat="1.3560000"', b'lat="91.0"')
        with pytest.raises(CotError, match=r"lat"):
            parse_cot(xml)

    def test_outbound_validation_too(self):
        """We validate what we EMIT — a bad track corrupts someone else's picture."""
        with pytest.raises(CotError):
            build_cot(hostile(lat=999.0))
        with pytest.raises(CotError, match="CoT atom"):
            build_cot(hostile(cot_type="not-a-cot-type"))


class TestGeodesy:
    def test_offset_then_measure_round_trips(self):
        lat, lon = offset_lat_lon(BASE_LAT, BASE_LON, 1200.0, 45.0)
        brg, rng = bearing_range(BASE_LAT, BASE_LON, lat, lon)
        assert rng == pytest.approx(1200.0, abs=1.0)
        assert brg == pytest.approx(45.0, abs=0.1)

    @pytest.mark.parametrize("bearing", [0.0, 90.0, 180.0, 270.0, 315.0])
    def test_cardinal_bearings_preserved(self, bearing):
        lat, lon = offset_lat_lon(BASE_LAT, BASE_LON, 800.0, bearing)
        brg, _ = bearing_range(BASE_LAT, BASE_LON, lat, lon)
        assert abs(((brg - bearing + 180.0) % 360.0) - 180.0) < 0.2

    def test_bearing_is_always_in_range(self):
        for b in range(0, 360, 17):
            lat, lon = offset_lat_lon(BASE_LAT, BASE_LON, 500.0, float(b))
            brg, _ = bearing_range(BASE_LAT, BASE_LON, lat, lon)
            assert 0.0 <= brg < 360.0

    def test_elevation_angle_geometry(self):
        assert elevation_angle(100.0, 100.0) == pytest.approx(45.0, abs=0.1)
        assert elevation_angle(1000.0, 0.0) == pytest.approx(0.0, abs=0.1)

    def test_elevation_clamped_to_schema_range(self):
        """slew_to_cue validates elevation in [-90, 90]; never emit outside it."""
        assert -90.0 <= elevation_angle(0.001, 10000.0) <= 90.0
        assert -90.0 <= elevation_angle(0.0, -10000.0) <= 90.0


class TestSlewRateLimiter:
    """D2 — anti-gear-stripping. Three stacked protections."""

    def _limiter(self, **kw):
        from tools.c2_bridge import SlewRateLimiter
        return SlewRateLimiter(**kw)

    def test_rate_is_capped(self):
        """100 offers in a tight loop must not produce 100 cues."""
        lim = self._limiter(max_hz=5.0)
        emitted = sum(1 for i in range(100) if lim.offer(90.0 + i * 3.0, 45.0) is not None)
        assert emitted <= 2, f"rate cap leaked {emitted} cues"

    def test_first_offer_emits_immediately(self):
        assert self._limiter().offer(120.0, 30.0) is not None

    def test_lowpass_damps_a_step_change(self):
        """A 90-degree jump must not become a 90-degree command.

        Inspect `current`, not `offer`'s return: the filter advances on every
        offer while the rate cap independently decides what reaches the wire.
        """
        lim = self._limiter(alpha=0.3)
        lim.offer(0.0, 0.0)
        lim.offer(90.0, 0.0)
        az, _ = lim.current
        assert 0.0 < az < 90.0, f"step passed through undamped: {az}"

    def test_wraps_the_short_way_around_north(self):
        """359 -> 1 must not sweep 358 degrees through the gimbal."""
        lim = self._limiter(alpha=0.5)
        lim.offer(359.0, 0.0)
        lim.offer(1.0, 0.0)
        az, _ = lim.current
        assert az > 355.0 or az < 5.0, f"took the long way: {az}"

    def test_deadband_suppresses_micro_updates(self):
        lim = self._limiter(max_hz=1000.0, alpha=1.0)
        lim.offer(90.0, 45.0)
        assert lim.offer(90.05, 45.02) is None

    def test_reset_clears_filter_state(self):
        """Switching target must not carry the previous bearing."""
        lim = self._limiter(alpha=0.3)
        lim.offer(10.0, 0.0)
        lim.reset()
        assert lim.current is None
        az, _ = lim.offer(200.0, 0.0)
        assert az == pytest.approx(200.0)

    def test_reset_also_clears_the_rate_cap(self):
        """A target switch must cue immediately, not wait out the old cap."""
        lim = self._limiter(max_hz=5.0)
        assert lim.offer(90.0, 10.0) is not None
        assert lim.offer(200.0, 10.0) is None      # capped, as intended
        lim.reset()
        assert lim.offer(200.0, 10.0) is not None  # new target cues at once


class TestThreatPriority:
    def test_priority_is_lowest_time_to_impact(self):
        from tools.c2_bridge import ENGAGEMENT_PERIMETER_M, FleetState

        fleet = FleetState(simulated_nodes=0)
        fleet.seed_swarm()
        for t in fleet.threats.values():
            t.distance_m = 300.0
        fleet.threats["THREAT_BETA_01"].speed_mps = 100.0  # fastest -> lowest TTI
        assert fleet.priority_threat().uid == "THREAT_BETA_01"

    def test_threats_outside_perimeter_are_not_prioritised(self):
        from tools.c2_bridge import FleetState

        fleet = FleetState(simulated_nodes=0)
        fleet.seed_swarm()
        assert fleet.priority_threat() is None, "1000m+ threats must not cue the gimbal"

    def test_neutralized_threats_drop_out(self):
        from tools.c2_bridge import FleetState

        fleet = FleetState(simulated_nodes=0)
        fleet.seed_swarm()
        for t in fleet.threats.values():
            t.distance_m = 100.0
            t.status = "NEUTRALIZED"
        assert fleet.priority_threat() is None
        assert math.isinf(next(iter(fleet.threats.values())).time_to_impact_s)

    def test_node_one_is_the_only_real_node(self):
        from tools.c2_bridge import FleetState

        fleet = FleetState(simulated_nodes=19)
        real = [n for n in fleet.nodes.values() if n.is_real]
        assert len(real) == 1 and real[0].node_id == "KILLSWITCH_NODE_01"
        assert len(fleet.nodes) == 20


# ---- ATAK hardening: remarks, unicast, stale pad, base, honest labels --------


def _window_s(payload: bytes) -> float:
    """Seconds between a CoT event's time and stale stamps."""
    import xml.etree.ElementTree as ET
    from datetime import datetime

    root = ET.fromstring(payload)
    fmt = "%Y-%m-%dT%H:%M:%S.%fZ"
    return (datetime.strptime(root.get("stale"), fmt)
            - datetime.strptime(root.get("time"), fmt)).total_seconds()


class TestRemarks:
    def test_remarks_round_trip(self):
        parsed = parse_cot(build_cot(hostile(remarks="SIMULATED THREAT | 312 m")))
        assert parsed.remarks == "SIMULATED THREAT | 312 m"

    def test_remarks_are_escaped(self):
        raw = build_cot(hostile(remarks='<script>&"'))
        assert b"<script>" not in raw
        assert parse_cot(raw).remarks == '<script>&"'

    def test_oversize_remarks_rejected_outbound(self):
        from helper.comms.cot import MAX_REMARKS_CHARS

        with pytest.raises(CotError, match="remarks"):
            build_cot(hostile(remarks="x" * (MAX_REMARKS_CHARS + 1)))

    def test_payload_without_remarks_still_parses(self):
        xml = (b'<?xml version="1.0"?><event uid="A1" type="a-h-A">'
               b'<point lat="1.3" lon="103.7"/></event>')
        assert parse_cot(xml).remarks == ""


class TestUnicastAndStalePad:
    def test_unicast_dest_defaults_to_the_tak_port(self):
        from tools.c2_bridge import parse_unicast_dest

        assert parse_unicast_dest("10.0.0.5") == ("10.0.0.5", 6969)
        assert parse_unicast_dest("10.0.0.5:4242") == ("10.0.0.5", 4242)

    @pytest.mark.parametrize("bad", [
        "999.1.1.1", "10.0.0.5:0", "10.0.0.5:70000", "10.0.0.5:abc",
        "phone.local", "239.2.3.1",
    ])
    def test_unicast_dest_rejects_unusable_values(self, bad):
        import argparse

        from tools.c2_bridge import parse_unicast_dest

        with pytest.raises(argparse.ArgumentTypeError):
            parse_unicast_dest(bad)

    def test_multicast_goes_first_then_each_unicast(self):
        from tools.c2_bridge import make_cot_socket

        sock, dests = make_cot_socket(False, None, [("10.0.0.5", 6969), ("10.0.0.6", 4242)])
        sock.close()
        assert dests == [("239.2.3.1", 6969), ("10.0.0.5", 6969), ("10.0.0.6", 4242)]
        sock, dests = make_cot_socket(True, None, [("10.0.0.5", 6969)])
        sock.close()
        assert dests == [("127.0.0.1", 18999), ("10.0.0.5", 6969)]

    def test_stale_pad_extends_the_window(self):
        from tools.c2_bridge import prepare_datagram

        ev = hostile(stale_seconds=4.0)
        assert _window_s(prepare_datagram(ev)) == pytest.approx(4.0, abs=0.01)
        assert _window_s(prepare_datagram(ev, 5.0)) == pytest.approx(9.0, abs=0.01)

    def test_stale_pad_is_capped_so_dead_tracks_cannot_linger(self):
        from tools.c2_bridge import MAX_STALE_PAD_S, prepare_datagram

        window = _window_s(prepare_datagram(hostile(stale_seconds=4.0), 500.0))
        assert window == pytest.approx(4.0 + MAX_STALE_PAD_S, abs=0.01)

    @pytest.mark.parametrize("bad", ["-1", "61", "nan", "soon"])
    def test_stale_pad_arg_rejects_out_of_range(self, bad):
        import argparse

        from tools.c2_bridge import parse_stale_pad

        with pytest.raises(argparse.ArgumentTypeError):
            parse_stale_pad(bad)


class TestMapBase:
    def test_default_base_is_the_demo_site(self):
        from helper.comms.cot import DEMO_SITE_LAT, DEMO_SITE_LON
        from tools.c2_bridge import FleetState

        n1 = FleetState(simulated_nodes=0).nodes["KILLSWITCH_NODE_01"]
        assert (n1.lat, n1.lon) == (DEMO_SITE_LAT, DEMO_SITE_LON)

    def test_base_moves_the_fleet_and_keeps_threat_bearings(self):
        from tools.c2_bridge import FleetState

        fleet = FleetState(simulated_nodes=0, base=(10.0, 20.0, 5.0))
        fleet.seed_swarm()
        n1 = fleet.nodes["KILLSWITCH_NODE_01"]
        assert (n1.lat, n1.lon, n1.hae) == (10.0, 20.0, 5.0)
        for t in fleet.threats.values():
            brg, _ = bearing_range(10.0, 20.0, *t.position())
            assert abs(((brg - t.bearing_deg + 180) % 360) - 180) < 0.5

    def test_base_arg_parsing(self):
        import argparse

        from tools.c2_bridge import parse_base

        assert parse_base("1.2966,103.7764") == (1.2966, 103.7764, 20.0)
        assert parse_base("1.2966,103.7764,35") == (1.2966, 103.7764, 35.0)
        for bad in ("91,0", "1,2,3,4", "a,b", "nan,1", "1"):
            with pytest.raises(argparse.ArgumentTypeError):
                parse_base(bad)


class TestHonestLabels:
    def _node1(self):
        from tools.c2_bridge import FleetState

        return FleetState(simulated_nodes=3).nodes["KILLSWITCH_NODE_01"]

    def test_real_node_reads_no_link_before_any_telemetry(self):
        ev = self._node1().to_cot()
        assert ev.callsign == "KILLSWITCH-01 [NO LINK]"
        assert ev.remarks == "REAL NODE | STATE NO LINK | TGT -"

    def test_real_node_mirrors_its_reported_state_and_target(self):
        import time

        n1 = self._node1()
        n1.reported, n1.last_seen = True, time.monotonic()
        n1.status, n1.target_id = "ENGAGE", "TRK-BETA_01"
        ev = n1.to_cot()
        assert ev.callsign == "KILLSWITCH-01 [ENGAGE]"
        assert ev.remarks == "REAL NODE | STATE ENGAGE | TGT TRK-BETA_01"

    def test_real_node_reads_no_link_once_silent(self):
        import time

        from tools.c2_bridge import NODE_LINK_TIMEOUT_S

        n1 = self._node1()
        n1.reported, n1.status, n1.target_id = True, "ENGAGE", "TRK-BETA_01"
        n1.last_seen = time.monotonic() - NODE_LINK_TIMEOUT_S - 1.0
        ev = n1.to_cot()
        assert ev.callsign == "KILLSWITCH-01 [NO LINK]"
        assert "TGT -" in ev.remarks, "a silent node must not advertise a live target"

    def test_oversized_telemetry_cannot_break_outbound_validation(self):
        import time

        n1 = self._node1()
        n1.reported, n1.last_seen = True, time.monotonic()
        n1.status, n1.target_id = "S" * 500, "T" * 500
        build_cot(n1.to_cot())  # must not raise

    def test_everything_else_says_simulated(self):
        from tools.c2_bridge import FleetState

        fleet = FleetState(simulated_nodes=3)
        fleet.seed_swarm()
        for node in fleet.nodes.values():
            if not node.is_real:
                assert node.to_cot().remarks == f"SIMULATED {node.kind}"
        for threat in fleet.threats.values():
            assert threat.to_cot().remarks.startswith("SIMULATED THREAT | ")

    def test_dashboard_reads_no_link_before_any_telemetry(self):
        from tools.c2_bridge import FleetState

        assert FleetState(simulated_nodes=0).node1_view() == ("NO LINK", {})

    def test_dashboard_drops_a_silent_nodes_frozen_telemetry(self):
        import time

        from tools.c2_bridge import NODE_LINK_TIMEOUT_S, FleetState

        fleet = FleetState(simulated_nodes=0)
        n1 = fleet.nodes["KILLSWITCH_NODE_01"]
        fleet.node1_state = n1.status = "ENGAGE"
        fleet.node1_telemetry = {"pan_deg": 45.0, "mock_actuator": False}
        n1.reported, n1.last_seen = True, time.monotonic()
        assert fleet.node1_view() == ("ENGAGE", {"pan_deg": 45.0, "mock_actuator": False})

        n1.last_seen = time.monotonic() - NODE_LINK_TIMEOUT_S - 1.0
        assert fleet.node1_view() == ("NO LINK", {}), \
            "a silent node's last numbers must not stay on the dashboard"

    def test_last_will_drops_the_map_to_no_link_at_once(self):
        """The node's MQTT last-will must end the link, not refresh it, on the
        map and on the dashboard alike."""
        import asyncio
        import json

        from tools.c2_bridge import FleetState, task_mqtt_consume

        class Msg:
            def __init__(self, body):
                self.payload = json.dumps(body).encode()

        class FakeClient:
            def __init__(self, bodies):
                self._msgs = [Msg(b) for b in bodies]

            async def subscribe(self, topic):
                return None

            def messages(self):
                msgs = self._msgs

                class Ctx:
                    async def __aenter__(self):
                        async def gen():
                            for m in msgs:
                                yield m
                        return gen()

                    async def __aexit__(self, *exc):
                        return False
                return Ctx()

        fleet = FleetState(simulated_nodes=0)
        live = {"state": "ENGAGE", "target_id": "TRK-BETA_01", "pan_deg": 45.0}
        asyncio.run(task_mqtt_consume(fleet, FakeClient([live])))
        assert fleet.nodes["KILLSWITCH_NODE_01"].to_cot().callsign == "KILLSWITCH-01 [ENGAGE]"
        assert fleet.node1_view() == ("ENGAGE", live)

        asyncio.run(task_mqtt_consume(fleet, FakeClient([{"event": "node_lost"}])))
        assert fleet.nodes["KILLSWITCH_NODE_01"].to_cot().callsign == "KILLSWITCH-01 [NO LINK]"
        assert fleet.node1_view() == ("NO LINK", {})


# ---- dashboard socket: output-only, and one stalled phone can't freeze it ----


class TestDashboardSocket:
    class Good:
        def __init__(self):
            self.got = []

        async def send(self, blob):
            self.got.append(blob)

    class Stalled:
        """A locked iPhone: socket open, never reads, send never returns."""

        async def send(self, blob):
            import asyncio
            await asyncio.sleep(3600)

    class Closed:
        async def send(self, blob):
            from websockets.exceptions import ConnectionClosed
            raise ConnectionClosed(None, None)

    class Broken:
        async def send(self, blob):
            raise RuntimeError("unexpected")

    def _run(self, clients, timeout_s=0.05):
        import asyncio

        from tools.c2_bridge import broadcast

        # Outer guard: a regression to an unbounded send fails here, not hangs.
        asyncio.run(asyncio.wait_for(broadcast(clients, "frame", timeout_s), 2.0))

    def test_a_stalled_client_is_dropped_and_the_rest_still_get_the_frame(self):
        import time

        good, stalled = self.Good(), self.Stalled()
        clients = {good, stalled}
        t0 = time.monotonic()
        self._run(clients)
        assert time.monotonic() - t0 < 1.0
        assert good.got == ["frame"]
        assert clients == {good}

    def test_a_closed_client_is_dropped(self):
        good, closed = self.Good(), self.Closed()
        clients = {good, closed}
        self._run(clients)
        assert clients == {good} and good.got == ["frame"]

    def test_an_unexpected_send_error_is_contained(self):
        good, broken = self.Good(), self.Broken()
        clients = {good, broken}
        self._run(clients)  # must not raise into the TaskGroup
        assert clients == {good} and good.got == ["frame"]

    def test_inbound_frames_are_discarded_and_the_client_unregistered(self):
        import asyncio
        import json

        from tools.c2_bridge import serve_client

        clients: set = set()
        seen_registered = []

        class FakeWS:
            def __aiter__(self):
                return self._frames()

            async def _frames(self):
                seen_registered.append(self in clients)
                yield json.dumps({"action": "authorise"})
                yield json.dumps({"action": "abort"})

        asyncio.run(serve_client(FakeWS(), clients))
        assert seen_registered == [True]
        assert clients == set()

    def test_a_dropped_connection_is_not_an_error(self):
        import asyncio

        from websockets.exceptions import ConnectionClosed

        from tools.c2_bridge import serve_client

        clients: set = set()

        class DropsMidway:
            def __aiter__(self):
                return self._frames()

            async def _frames(self):
                yield "x"
                raise ConnectionClosed(None, None)

        asyncio.run(serve_client(DropsMidway(), clients))
        assert clients == set()


# ---- which address to tell a phone to open -----------------------------------


class TestPhoneAddresses:
    """order_candidates decides the URL the bridge prints for a phone."""

    def test_the_real_demo_laptop(self):
        # Adapter list from the Windows rig on the iPhone hotspot: three
        # unconfigured adapters with APIPA addresses, and the Wi-Fi.
        from tools.multicast_test import order_candidates

        rig = ["169.254.185.249", "169.254.246.177", "172.20.10.9", "169.254.196.224"]
        assert order_candidates(rig, "172.20.10.9") == ["172.20.10.9"]

    def test_default_route_moves_first_even_when_already_listed(self):
        from tools.multicast_test import order_candidates

        assert order_candidates(["10.0.0.5", "192.168.1.20"], "192.168.1.20") == [
            "192.168.1.20", "10.0.0.5"]

    def test_loopback_and_duplicates_are_dropped(self):
        from tools.multicast_test import order_candidates

        assert order_candidates(["127.0.0.1", "10.0.0.5", "10.0.0.5"], None) == ["10.0.0.5"]

    def test_link_local_default_is_dropped_but_others_kept(self):
        from tools.multicast_test import order_candidates

        assert order_candidates(["10.0.0.5"], "169.254.9.9") == ["10.0.0.5"]

    def test_garbage_and_empty_input(self):
        from tools.multicast_test import order_candidates

        assert order_candidates([], None) == []
        assert order_candidates(["not-an-ip", "::1"], None) == []
