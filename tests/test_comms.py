"""C2 payload validation tests. No broker, no network."""
from __future__ import annotations

import json

import pytest

from helper.comms.schemas import (
    TOPIC_OPERATOR_AUTH,
    TOPIC_SLEW_TO_CUE,
    ValidationError,
    parse_operator_auth,
    parse_slew_to_cue,
)


def cue(**overrides) -> bytes:
    body = {"azimuth": 135.0, "elevation": 20.0, "target_id": "TRK-0042"}
    body.update(overrides)
    return json.dumps(body).encode()


def auth(**overrides) -> bytes:
    body = {"auth": True, "target_id": "TRK-0042", "token": "s3cret", "nonce": "n1"}
    body.update(overrides)
    return json.dumps(body).encode()


class TestSlewToCue:
    def test_valid_cue_parses(self):
        result = parse_slew_to_cue(cue())
        assert (result.azimuth, result.elevation, result.target_id) == (
            135.0, 20.0, "TRK-0042",
        )

    @pytest.mark.parametrize("azimuth", [-1.0, 360.0, 400.0, 1e9])
    def test_azimuth_out_of_range_rejected(self, azimuth):
        with pytest.raises(ValidationError, match="azimuth"):
            parse_slew_to_cue(cue(azimuth=azimuth))

    @pytest.mark.parametrize("elevation", [-90.1, 90.1, -180.0])
    def test_elevation_out_of_range_rejected(self, elevation):
        with pytest.raises(ValidationError, match="elevation"):
            parse_slew_to_cue(cue(elevation=elevation))

    def test_elevation_bounds_are_inclusive(self):
        assert parse_slew_to_cue(cue(elevation=90.0)).elevation == 90.0
        assert parse_slew_to_cue(cue(elevation=-90.0)).elevation == -90.0

    def test_non_finite_rejected(self):
        # NaN and Infinity are legal in Python's json but must never reach a
        # PWM calculation.
        for literal in ("NaN", "Infinity", "-Infinity"):
            payload = ('{"azimuth": %s, "elevation": 0, "target_id": "T1"}' % literal)
            with pytest.raises(ValidationError, match="finite"):
                parse_slew_to_cue(payload.encode())

    def test_bool_is_not_a_number(self):
        """isinstance(True, int) is True in Python; a naive check would pass."""
        with pytest.raises(ValidationError, match="must be a number"):
            parse_slew_to_cue(cue(azimuth=True))

    def test_string_azimuth_rejected(self):
        with pytest.raises(ValidationError, match="must be a number"):
            parse_slew_to_cue(cue(azimuth="135"))

    def test_missing_field_rejected(self):
        with pytest.raises(ValidationError, match="missing required"):
            parse_slew_to_cue(b'{"azimuth": 1.0, "elevation": 2.0}')

    def test_unknown_field_rejected(self):
        with pytest.raises(ValidationError, match="unknown fields"):
            parse_slew_to_cue(cue(extra="surprise"))

    @pytest.mark.parametrize(
        "target_id", ["", "a" * 65, "drop table;", "../../etc/passwd", "T 1"]
    )
    def test_hostile_target_id_rejected(self, target_id):
        with pytest.raises(ValidationError, match="target_id"):
            parse_slew_to_cue(cue(target_id=target_id))

    def test_malformed_json_rejected(self):
        with pytest.raises(ValidationError, match="not valid JSON"):
            parse_slew_to_cue(b"{not json")

    def test_non_object_rejected(self):
        with pytest.raises(ValidationError, match="must be a JSON object"):
            parse_slew_to_cue(b"[1, 2, 3]")

    def test_oversize_payload_rejected_before_parsing(self):
        with pytest.raises(ValidationError, match="exceeds"):
            parse_slew_to_cue(b"x" * 5000)

    def test_invalid_utf8_rejected(self):
        with pytest.raises(ValidationError, match="UTF-8"):
            parse_slew_to_cue(b"\xff\xfe\x00")


class TestOperatorAuth:
    def test_valid_auth_parses(self):
        result = parse_operator_auth(auth(), expected_token="s3cret")
        assert result.auth is True
        assert result.target_id == "TRK-0042"

    def test_explicit_denial_is_valid(self):
        """auth=False is a decision the state machine must honour, not an error."""
        assert parse_operator_auth(auth(auth=False), "s3cret").auth is False

    def test_wrong_token_rejected(self):
        with pytest.raises(ValidationError, match="token mismatch"):
            parse_operator_auth(auth(token="guess"), expected_token="s3cret")

    def test_token_not_echoed_in_error(self):
        """A rejection message must not leak the supplied secret into logs."""
        with pytest.raises(ValidationError) as exc:
            parse_operator_auth(auth(token="hunter2"), expected_token="s3cret")
        assert "hunter2" not in str(exc.value)

    def test_non_bool_auth_rejected(self):
        for value in (1, "true", "yes", None):
            with pytest.raises(ValidationError, match="auth must be a boolean"):
                parse_operator_auth(auth(auth=value), "s3cret")

    def test_nonce_required_for_replay_protection(self):
        payload = json.dumps(
            {"auth": True, "target_id": "T1", "token": "s3cret"}
        ).encode()
        with pytest.raises(ValidationError, match="missing required"):
            parse_operator_auth(payload, "s3cret")

    def test_empty_nonce_rejected(self):
        with pytest.raises(ValidationError, match="nonce"):
            parse_operator_auth(auth(nonce=""), "s3cret")

    def test_token_check_skipped_when_none_configured(self):
        assert parse_operator_auth(auth(token="anything"), expected_token=None).auth


class TestMockC2Validation:
    def test_mock_client_runs_real_validators(self):
        """Injected raw payloads must not bypass validation."""
        from helper.comms.mqtt_client import MockC2Client

        client = MockC2Client(auth_token="s3cret")
        assert client.inject_raw(TOPIC_SLEW_TO_CUE, cue()) is True
        assert client.inject_raw(TOPIC_SLEW_TO_CUE, cue(azimuth=999.0)) is False
        assert client.inject_raw(TOPIC_OPERATOR_AUTH, auth(token="wrong")) is False

        assert client.poll() is not None
        assert client.poll() is None
        assert len(client.rejected) == 2
