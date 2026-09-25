"""C2 payload validation.

The legacy handler that could switch on the effector carried the comment
"Only valid messages should be received, so no need to make checks." Every
inbound payload is hostile until proven otherwise. See guardrails.md section 5.

Validation is pure-Python and dependency-free so it is testable without a broker.
"""
from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass
from typing import Any, Dict, Optional

# Cap before parsing. An unbounded payload is a memory-exhaustion vector.
MAX_PAYLOAD_BYTES: int = 4096

# target_id charset: conservative on purpose. These strings end up in log lines,
# filenames and operator displays.
TARGET_ID_PATTERN = re.compile(r"^[A-Za-z0-9_\-]{1,64}$")

TOPIC_SLEW_TO_CUE: str = "c2/radar/slew_to_cue"
TOPIC_OPERATOR_AUTH: str = "c2/operator/auth"
TOPIC_NODE_TELEMETRY: str = "c2/node/telemetry"
TOPIC_NODE_EVENT: str = "c2/node/event"
# Operator tasking of SIMULATED fleet assets. Consumed by the bridge only; the
# node never subscribes, so nothing on this topic can reach the effector.
TOPIC_OPERATOR_TASK: str = "c2/operator/task"

# Keys 1..N on the operator console. Three simulated assets: the 1:3 story.
MAX_TASK_ASSETS: int = 3


class ValidationError(ValueError):
    """Payload rejected. Carries a reason suitable for the audit log."""


@dataclass(frozen=True)
class SlewToCue:
    """A validated radar cue. Transitions IDLE -> SCAN."""

    azimuth: float
    elevation: float
    target_id: str


@dataclass(frozen=True)
class OperatorAuth:
    """A validated operator decision.

    Carries target_id so authorisation binds to the track it was granted for;
    an auth for track 7 must never fire on track 9. Carries nonce so the state
    machine can enforce single use.
    """

    auth: bool
    target_id: str
    token: str
    nonce: str


@dataclass(frozen=True)
class OperatorTask:
    """A validated operator tasking of simulated asset `asset` (1..MAX_TASK_ASSETS).

    It labels a SIMULATED asset on the map as tasked by a human. It never
    engages anything and never addresses the real node.
    """

    asset: int
    token: str


def _decode(payload: bytes) -> Dict[str, Any]:
    """Size-cap, decode and parse. Raises ValidationError on any failure."""
    if len(payload) > MAX_PAYLOAD_BYTES:
        raise ValidationError(
            f"payload {len(payload)} bytes exceeds {MAX_PAYLOAD_BYTES} cap"
        )
    try:
        text = payload.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValidationError(f"payload is not valid UTF-8: {exc}") from exc

    try:
        obj = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ValidationError(f"payload is not valid JSON: {exc}") from exc

    if not isinstance(obj, dict):
        raise ValidationError(f"payload must be a JSON object, got {type(obj).__name__}")
    return obj


def _require_keys(obj: Dict[str, Any], required: set, context: str) -> None:
    """Reject missing and unknown keys. Unknown keys mean schema drift."""
    keys = set(obj.keys())
    missing = required - keys
    if missing:
        raise ValidationError(f"{context}: missing required {sorted(missing)}")
    unknown = keys - required
    if unknown:
        raise ValidationError(f"{context}: unknown fields {sorted(unknown)}")


def _finite_float(obj: Dict[str, Any], key: str, lo: float, hi: float) -> float:
    """Extract a finite float in [lo, hi).

    bool is rejected explicitly: in Python `isinstance(True, int)` is True, so a
    naive numeric check would accept `{"azimuth": true}` as 1.0.
    """
    value = obj[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValidationError(f"{key} must be a number, got {type(value).__name__}")
    value = float(value)
    if not math.isfinite(value):
        raise ValidationError(f"{key} must be finite, got {value}")
    if not (lo <= value < hi):
        raise ValidationError(f"{key}={value} outside [{lo}, {hi})")
    return value


def _target_id(obj: Dict[str, Any], key: str = "target_id") -> str:
    value = obj[key]
    if not isinstance(value, str):
        raise ValidationError(f"{key} must be a string, got {type(value).__name__}")
    if not TARGET_ID_PATTERN.match(value):
        raise ValidationError(
            f"{key}={value!r} must be 1-64 chars of [A-Za-z0-9_-]"
        )
    return value


def parse_slew_to_cue(payload: bytes) -> SlewToCue:
    """Validate a radar cue.

    Args:
        payload: Raw MQTT payload bytes.

    Returns:
        A validated SlewToCue.

    Raises:
        ValidationError: On any schema, type or range failure. The caller logs
            and discards; a partially-applied cue is never produced.
    """
    obj = _decode(payload)
    _require_keys(obj, {"azimuth", "elevation", "target_id"}, "slew_to_cue")
    return SlewToCue(
        azimuth=_finite_float(obj, "azimuth", 0.0, 360.0),
        # Upper bound nudged so exactly 90.0 is accepted by the half-open check.
        elevation=_finite_float(obj, "elevation", -90.0, 90.0 + 1e-9),
        target_id=_target_id(obj),
    )


def parse_operator_auth(payload: bytes, expected_token: Optional[str] = None) -> OperatorAuth:
    """Validate an operator authorisation.

    Args:
        payload: Raw MQTT payload bytes.
        expected_token: Shared secret. When provided, a mismatch is rejected.

    Returns:
        A validated OperatorAuth. Note that `auth=False` is a valid message —
        an explicit denial, which the state machine must honour.

    Raises:
        ValidationError: On schema failure or token mismatch.
    """
    obj = _decode(payload)
    _require_keys(obj, {"auth", "target_id", "token", "nonce"}, "operator_auth")

    if not isinstance(obj["auth"], bool):
        raise ValidationError(f"auth must be a boolean, got {type(obj['auth']).__name__}")
    for key in ("token", "nonce"):
        if not isinstance(obj[key], str) or not obj[key]:
            raise ValidationError(f"{key} must be a non-empty string")
        if len(obj[key]) > 256:
            raise ValidationError(f"{key} exceeds 256 chars")

    if expected_token is not None and obj["token"] != expected_token:
        # Do not echo the supplied token into logs.
        raise ValidationError("token mismatch")

    return OperatorAuth(
        auth=obj["auth"],
        target_id=_target_id(obj),
        token=obj["token"],
        nonce=obj["nonce"],
    )


def parse_operator_task(payload: bytes, expected_token: Optional[str] = None) -> OperatorTask:
    """Validate an operator tasking message.

    Args:
        payload: Raw MQTT payload bytes.
        expected_token: Shared secret, the same one operator auth uses. When
            provided, a mismatch is rejected.

    Returns:
        A validated OperatorTask.

    Raises:
        ValidationError: On schema failure, an asset outside 1..MAX_TASK_ASSETS,
            or a token mismatch.
    """
    obj = _decode(payload)
    _require_keys(obj, {"asset", "token"}, "operator_task")

    asset = obj["asset"]
    # bool is an int in Python; {"asset": true} must not mean asset 1.
    if isinstance(asset, bool) or not isinstance(asset, int):
        raise ValidationError(f"asset must be an integer, got {type(asset).__name__}")
    if not 1 <= asset <= MAX_TASK_ASSETS:
        raise ValidationError(f"asset={asset} outside 1..{MAX_TASK_ASSETS}")
    token = obj["token"]
    if not isinstance(token, str) or not token or len(token) > 256:
        raise ValidationError("token must be a non-empty string of at most 256 chars")
    if expected_token is not None and token != expected_token:
        raise ValidationError("token mismatch")

    return OperatorTask(asset=asset, token=token)
