"""Engagement state machine — the single authority on system state.

Every transition passes through one guarded method holding an RLock. No
component assigns state directly (CLAUDE.md rule 2, guardrails section 1 HARD).
This replaces the legacy `global_data["state"] = x` pattern, which was mutated
from three threads with no synchronisation.
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Dict, FrozenSet, Optional, Sequence, Tuple

from helper.vision.types import AimPoint, Detection

logger = logging.getLogger(__name__)


class EngagementState(str, Enum):
    """The six states. str-mixin so values serialise directly into MQTT events."""

    IDLE = "IDLE"
    SCAN = "SCAN"
    TRACK = "TRACK"
    HOLD = "HOLD"
    OPERATOR_AUTH = "OPERATOR_AUTH"
    ENGAGE = "ENGAGE"


# Declared transitions. Anything absent raises.
#
# Two invariants encoded here:
#   1. ENGAGE has exactly one predecessor, OPERATOR_AUTH. Non-negotiable.
#   2. IDLE is reachable from everywhere — the universal safe harbour.
LEGAL_TRANSITIONS: Dict[EngagementState, FrozenSet[EngagementState]] = {
    EngagementState.IDLE: frozenset({EngagementState.SCAN}),
    EngagementState.SCAN: frozenset({EngagementState.TRACK, EngagementState.IDLE}),
    EngagementState.TRACK: frozenset(
        {EngagementState.HOLD, EngagementState.SCAN, EngagementState.IDLE}
    ),
    EngagementState.HOLD: frozenset(
        {
            EngagementState.OPERATOR_AUTH,
            EngagementState.TRACK,
            EngagementState.SCAN,
            EngagementState.IDLE,
        }
    ),
    EngagementState.OPERATOR_AUTH: frozenset(
        {
            EngagementState.ENGAGE,
            EngagementState.TRACK,
            EngagementState.SCAN,
            EngagementState.IDLE,
        }
    ),
    EngagementState.ENGAGE: frozenset({EngagementState.IDLE}),
}


class IllegalTransitionError(RuntimeError):
    """An undeclared transition was attempted. The caller forces IDLE."""


@dataclass(frozen=True)
class Transition:
    """One entry in the audit trail."""

    from_state: EngagementState
    to_state: EngagementState
    reason: str
    at: float


class StateMachine:
    """Guarded state with an audit trail.

    Thread-safety: all public methods are safe from any thread. The lock is
    reentrant so an on_transition callback may read state without deadlocking.
    """

    def __init__(
        self,
        on_transition: Optional[Callable[[Transition], None]] = None,
        initial: EngagementState = EngagementState.IDLE,
    ) -> None:
        """Args:
            on_transition: Called after every accepted transition, holding the
                lock. Must not block — publish asynchronously, do not do I/O.
            initial: Starting state.
        """
        self._lock = threading.RLock()
        self._state = initial
        self._entered_at = time.monotonic()
        self._on_transition = on_transition
        self._history: list = []

    @property
    def state(self) -> EngagementState:
        with self._lock:
            return self._state

    @property
    def time_in_state(self) -> float:
        """Seconds since entering the current state. Uses monotonic time, so an
        NTP step cannot extend or truncate a burn."""
        with self._lock:
            return time.monotonic() - self._entered_at

    def is_in(self, *states: EngagementState) -> bool:
        with self._lock:
            return self._state in states

    def transition_to(self, target: EngagementState, reason: str) -> Transition:
        """Move to `target`.

        Args:
            target: Desired state.
            reason: Why — recorded in the audit trail and published to C2.

        Returns:
            The recorded Transition.

        Raises:
            IllegalTransitionError: If the transition is not declared. The
                caller must respond by forcing IDLE.
        """
        with self._lock:
            current = self._state
            if target == current:
                # Re-entry is a no-op, not an error: handlers may re-assert.
                return Transition(current, target, f"re-entry: {reason}", time.monotonic())

            if target not in LEGAL_TRANSITIONS[current]:
                raise IllegalTransitionError(
                    f"{current.value} -> {target.value} is not declared "
                    f"(legal: {sorted(s.value for s in LEGAL_TRANSITIONS[current])})"
                )

            now = time.monotonic()
            held_for = now - self._entered_at
            transition = Transition(current, target, reason, now)
            self._state = target
            self._entered_at = now
            self._history.append(transition)

            logger.info(
                "%s -> %s (%.2fs in previous) | %s",
                current.value, target.value, held_for, reason,
            )
            if self._on_transition is not None:
                self._on_transition(transition)
            return transition

    def force_idle(self, reason: str) -> Transition:
        """Unconditional transition to IDLE, bypassing the table.

        The only bypass in the system, and it only ever moves toward safety.
        Used by fault handlers, where refusing to transition would leave the
        node stuck in an armed state.
        """
        with self._lock:
            current = self._state
            now = time.monotonic()
            transition = Transition(current, EngagementState.IDLE, f"FORCED: {reason}", now)
            self._state = EngagementState.IDLE
            self._entered_at = now
            self._history.append(transition)
            logger.warning("FORCED %s -> IDLE | %s", current.value, reason)
            if self._on_transition is not None:
                self._on_transition(transition)
            return transition

    def history(self) -> Sequence[Transition]:
        with self._lock:
            return tuple(self._history)


@dataclass(frozen=True)
class TrackSnapshot:
    """One completed pass of the vision pipeline.

    Immutable and timestamped so the state machine can reason about staleness
    and compute true glass-to-command latency.
    """

    frame_id: int
    captured_at: float
    processed_at: float
    detections: Tuple[Detection, ...] = field(default_factory=tuple)
    target: Optional[Detection] = None
    aim: Optional[AimPoint] = None
    error_px: float = float("inf")

    @property
    def has_target(self) -> bool:
        return self.target is not None

    @property
    def pipeline_latency_s(self) -> float:
        """Capture to inference-complete, in seconds."""
        return self.processed_at - self.captured_at

    def age_s(self, now: Optional[float] = None) -> float:
        """Seconds since inference completed."""
        return (now if now is not None else time.monotonic()) - self.processed_at


class SnapshotHolder:
    """Single-slot thread-safe handoff from the vision worker to the brain.

    Overwrite-not-queue, for the same reason the frame grabber overwrites: a
    queued result is a stale result, and stale results drive the gimbal to
    where the target used to be.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._snapshot: Optional[TrackSnapshot] = None

    def publish(self, snapshot: TrackSnapshot) -> None:
        """Called from the vision worker thread."""
        with self._lock:
            self._snapshot = snapshot

    def latest(self) -> Optional[TrackSnapshot]:
        """Called from the state machine thread. Never blocks on inference."""
        with self._lock:
            return self._snapshot

    def clear(self) -> None:
        """Drop the held snapshot. Called on entry to IDLE so a stale target
        cannot survive into the next engagement."""
        with self._lock:
            self._snapshot = None
