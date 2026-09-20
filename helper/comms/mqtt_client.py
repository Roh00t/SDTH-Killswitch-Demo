"""C2 plane client — cueing and authorisation only.

MQTT is never in the inner control loop. It delivers cues and authorisations and
carries telemetry out; the tracking loop runs on serial and does not wait on a
broker.

Threading contract (guardrails.md section 1, HARD): paho dispatches callbacks on
its own network thread. Callbacks here validate, enqueue and return. They never
transition state, drive the actuator, or perform blocking I/O. A blocked callback
stalls the entire MQTT client.
"""
from __future__ import annotations

import json
import logging
import queue
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple, Union

import paho.mqtt.client as mqtt

from helper.comms.schemas import (
    TOPIC_NODE_EVENT,
    TOPIC_NODE_TELEMETRY,
    TOPIC_OPERATOR_AUTH,
    TOPIC_SLEW_TO_CUE,
    OperatorAuth,
    SlewToCue,
    ValidationError,
    parse_operator_auth,
    parse_slew_to_cue,
)

logger = logging.getLogger(__name__)

# Bounded: backpressure must surface as dropped stale cues, not memory growth.
INBOX_MAXSIZE: int = 64

C2Message = Union[SlewToCue, OperatorAuth]


@dataclass(frozen=True)
class RejectionStats:
    """Validation failure counters. A spike is an attack indicator."""

    slew_rejected: int
    auth_rejected: int
    total_received: int


class C2Client:
    """Validated MQTT client for the C2 plane.

    Thread-safety: `poll()` is safe from the state machine thread. `publish_*`
    are safe from any thread. Internal counters are lock-guarded.
    """

    def __init__(
        self,
        broker_host: str,
        broker_port: int = 1883,
        node_id: str = "killswitch-01",
        auth_token: Optional[str] = None,
        keepalive: int = 30,
    ) -> None:
        """Configure the client. Does not connect; call `connect()`.

        Args:
            broker_host: Broker address.
            broker_port: Broker port.
            node_id: This node's identity, used as MQTT client id and in events.
            auth_token: Shared secret required on operator auth messages. None
                disables token checking — acceptable only on an isolated demo
                network, and it must be stated as such.
            keepalive: MQTT keepalive seconds.
        """
        self._node_id = node_id
        self._broker = (broker_host, broker_port)
        self._auth_token = auth_token
        self._keepalive = keepalive

        self._inbox: "queue.Queue[C2Message]" = queue.Queue(maxsize=INBOX_MAXSIZE)
        self._lock = threading.Lock()
        self._slew_rejected = 0
        self._auth_rejected = 0
        self._total_received = 0
        self._connected = threading.Event()

        if auth_token is None:
            logger.warning(
                "No auth_token configured: operator auth tokens will not be "
                "verified. Acceptable only on an isolated demo network."
            )

        # paho-mqtt <2.0 callback signatures. Do not unpin; 2.x changed these.
        self._client = mqtt.Client(client_id=node_id, clean_session=True)
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message

        # Last will: if this node dies, C2 learns immediately rather than
        # inferring it from telemetry silence.
        self._client.will_set(
            TOPIC_NODE_EVENT,
            json.dumps({"node_id": node_id, "event": "node_lost", "ts": None}),
            qos=1,
            retain=False,
        )

    # ---- connection lifecycle -------------------------------------------

    def connect(self, timeout: float = 5.0) -> None:
        """Connect and start the network loop.

        Raises:
            ConnectionError: If the broker is unreachable or does not confirm
                within the timeout.
        """
        host, port = self._broker
        try:
            self._client.connect(host, port, keepalive=self._keepalive)
        except (OSError, ValueError) as exc:
            raise ConnectionError(
                f"Could not reach broker {host}:{port}: {exc}. Start it with "
                f"'/opt/homebrew/opt/mosquitto/sbin/mosquitto -v' (macOS) or "
                f"'sudo systemctl start mosquitto' (Linux), and leave it running "
                f"in its own terminal."
            ) from exc

        self._client.loop_start()
        if not self._connected.wait(timeout=timeout):
            self._client.loop_stop()
            raise ConnectionError(f"Broker {host}:{port} did not confirm within {timeout}s")
        logger.info("C2 connected to %s:%d as %s", host, port, self._node_id)

    def disconnect(self) -> None:
        """Stop the network loop and disconnect. Idempotent."""
        self._connected.clear()
        self._client.loop_stop()
        self._client.disconnect()
        logger.info("C2 disconnected")

    @property
    def is_connected(self) -> bool:
        """False after a broker disconnect; the state machine treats this as a
        fault and returns to IDLE. No cue means no engagement."""
        return self._connected.is_set()

    # ---- paho callbacks (network thread — validate and enqueue only) -----

    def _on_connect(self, client, userdata, flags, rc) -> None:
        if rc != 0:
            logger.error("Broker refused connection, rc=%d", rc)
            return
        # Subscribe inside on_connect so subscriptions survive a reconnect.
        client.subscribe([(TOPIC_SLEW_TO_CUE, 1), (TOPIC_OPERATOR_AUTH, 1)])
        self._connected.set()

    def _on_disconnect(self, client, userdata, rc) -> None:
        self._connected.clear()
        if rc != 0:
            logger.error("Unexpected broker disconnect, rc=%d", rc)

    def _on_message(self, client, userdata, msg: mqtt.MQTTMessage) -> None:
        """Validate and enqueue. Never acts. Runs on paho's network thread."""
        with self._lock:
            self._total_received += 1

        try:
            if msg.topic == TOPIC_SLEW_TO_CUE:
                parsed: C2Message = parse_slew_to_cue(msg.payload)
            elif msg.topic == TOPIC_OPERATOR_AUTH:
                parsed = parse_operator_auth(msg.payload, self._auth_token)
            else:
                logger.warning("Message on unsubscribed topic %s, discarded", msg.topic)
                return
        except ValidationError as exc:
            with self._lock:
                if msg.topic == TOPIC_SLEW_TO_CUE:
                    self._slew_rejected += 1
                else:
                    self._auth_rejected += 1
            logger.warning(
                "REJECTED %s: %s | raw=%.200r", msg.topic, exc, msg.payload
            )
            return

        try:
            self._inbox.put_nowait(parsed)
        except queue.Full:
            # Drop rather than block: blocking here stalls the MQTT client.
            logger.error("C2 inbox full (%d); dropping %s", INBOX_MAXSIZE, msg.topic)

    # ---- consumer side (state machine thread) ---------------------------

    def poll(self, timeout: float = 0.0) -> Optional[C2Message]:
        """Take the next validated message, or None.

        Args:
            timeout: Seconds to wait. 0.0 returns immediately.

        Returns:
            A validated SlewToCue or OperatorAuth, or None if none pending.
        """
        try:
            if timeout <= 0.0:
                return self._inbox.get_nowait()
            return self._inbox.get(timeout=timeout)
        except queue.Empty:
            return None

    def drain(self) -> int:
        """Discard all pending messages, returning the count dropped.

        Called on entry to IDLE so a cue queued during an engagement cannot
        immediately re-trigger one.
        """
        dropped = 0
        while self.poll() is not None:
            dropped += 1
        return dropped

    # ---- outbound -------------------------------------------------------

    def publish_telemetry(self, state: str, detail: dict) -> None:
        """Publish node telemetry. Best-effort, QoS 0 — never blocks the loop."""
        self._publish(TOPIC_NODE_TELEMETRY, {"state": state, **detail}, qos=0)

    def publish_event(self, event: str, detail: dict) -> None:
        """Publish an audit event at QoS 1.

        State transitions, engagements and faults go here. The audit trail is a
        deliverable, so these are delivered at least once, unlike telemetry.
        """
        self._publish(TOPIC_NODE_EVENT, {"event": event, **detail}, qos=1)

    def publish_raw(self, topic: str, payload: bytes, qos: int = 0) -> None:
        """Publish raw bytes (the video link). QoS 0 — never blocks the loop."""
        try:
            self._client.publish(topic, payload, qos=qos)
        except (OSError, ValueError) as exc:
            logger.debug("Raw publish to %s failed: %s", topic, exc)

    def _publish(self, topic: str, body: dict, qos: int) -> None:
        payload = json.dumps({"node_id": self._node_id, "ts": time.time(), **body})
        try:
            self._client.publish(topic, payload, qos=qos)
        except (OSError, ValueError) as exc:
            # Telemetry failure must never take down the control loop.
            logger.warning("Publish to %s failed: %s", topic, exc)

    # ---- diagnostics ----------------------------------------------------

    def stats(self) -> RejectionStats:
        """Snapshot of validation counters."""
        with self._lock:
            return RejectionStats(
                slew_rejected=self._slew_rejected,
                auth_rejected=self._auth_rejected,
                total_received=self._total_received,
            )


class MockC2Client:
    """In-memory C2 for hardware-free tests.

    Second implementation of the C2 surface. Messages are injected directly,
    bypassing the broker but NOT bypassing validation — `inject_raw` runs the
    same parsers, so tests exercise the real rejection logic.
    """

    def __init__(self, auth_token: Optional[str] = None) -> None:
        self._inbox: "queue.Queue[C2Message]" = queue.Queue()
        self._auth_token = auth_token
        self.published: list = []
        self.rejected: list = []
        self._connected = True

    def connect(self, timeout: float = 5.0) -> None:
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    def inject(self, message: C2Message) -> None:
        """Enqueue an already-valid message."""
        self._inbox.put(message)

    def inject_raw(self, topic: str, payload: bytes) -> bool:
        """Feed raw bytes through the real validators.

        Returns:
            True if accepted and enqueued, False if rejected.
        """
        try:
            if topic == TOPIC_SLEW_TO_CUE:
                self._inbox.put(parse_slew_to_cue(payload))
            elif topic == TOPIC_OPERATOR_AUTH:
                self._inbox.put(parse_operator_auth(payload, self._auth_token))
            else:
                return False
            return True
        except ValidationError as exc:
            self.rejected.append((topic, str(exc)))
            return False

    def poll(self, timeout: float = 0.0) -> Optional[C2Message]:
        try:
            if timeout <= 0.0:
                return self._inbox.get_nowait()
            return self._inbox.get(timeout=timeout)
        except queue.Empty:
            return None

    def drain(self) -> int:
        dropped = 0
        while self.poll() is not None:
            dropped += 1
        return dropped

    def publish_telemetry(self, state: str, detail: dict) -> None:
        self.published.append(("telemetry", state, detail))

    def publish_event(self, event: str, detail: dict) -> None:
        self.published.append(("event", event, detail))

    def publish_raw(self, topic: str, payload: bytes, qos: int = 0) -> None:
        self.published.append(("raw", topic, len(payload)))
