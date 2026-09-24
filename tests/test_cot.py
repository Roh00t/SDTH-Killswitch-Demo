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
