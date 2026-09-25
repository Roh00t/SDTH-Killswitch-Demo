"""Operator console — the C2 dashboard and the human authorisation gate.

The node runs find-track-hold autonomously at machine speed. This is where a
human sits in the loop: the node reaches HOLD, publishes a firing solution, and
waits. Nothing engages until someone here presses SPACE.

That gate is not a limitation to apologise for. It is the ROE compliance story,
and it is the reason this architecture is presentable to a defence audience.

Controls:
    SPACE   authorise engagement (only while the node is in OPERATOR_AUTH)
    D       deny engagement
    C       send a test radar cue
    1/2/3   task SIMULATED asset 1/2/3 onto the next unassigned threat (map
            label only: it never engages and never addresses the real node)
    Q/ESC   quit the console (the node keeps running)

Keys reach this OpenCV window only while it has focus. Click its title bar,
not the terminal, before the node asks for authorisation.

Usage:
    python -m tools.operator_console
    python -m tools.operator_console --broker 192.168.1.50 --config config/bench.yaml
"""
from __future__ import annotations

import argparse
import json
import logging
import queue
import sys
import time
import uuid
from typing import Any, Dict, Optional

import cv2
import numpy as np
import paho.mqtt.client as mqtt
import yaml

from helper.comms.schemas import (
    TOPIC_NODE_EVENT,
    TOPIC_NODE_TELEMETRY,
    TOPIC_OPERATOR_AUTH,
    TOPIC_OPERATOR_TASK,
    TOPIC_SLEW_TO_CUE,
)
from helper.comms.video import TOPIC_NODE_VIDEO, decode_jpeg

logger = logging.getLogger("console")

PANEL_W, PANEL_H = 980, 620
_WHITE = (245, 245, 245)
_DIM = (140, 140, 140)
_GREEN = (80, 220, 80)
_AMBER = (60, 180, 250)
_RED = (60, 60, 240)
_BG = (24, 24, 28)

_STATE_COLOUR = {
    "IDLE": _DIM, "SCAN": _AMBER, "TRACK": _GREEN,
    "HOLD": _GREEN, "OPERATOR_AUTH": _AMBER, "ENGAGE": _RED,
}


class OperatorConsole:
    """MQTT-driven dashboard. Owns no hardware; it watches a remote node."""

    def __init__(self, broker: str, port: int, auth_token: Optional[str]) -> None:
        self._auth_token = auth_token
        self._frames: "queue.Queue[np.ndarray]" = queue.Queue(maxsize=2)
        self._last_frame: Optional[np.ndarray] = None
        self._state: str = "UNKNOWN"
        self._telemetry: Dict[str, Any] = {}
        self._events: list = []
        self._last_event_at: float = 0.0
        self._authorised_nonces: set = set()
        self._connected = False

        self._client = mqtt.Client(client_id=f"operator-{uuid.uuid4().hex[:6]}")
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._broker = (broker, port)

    # ---- mqtt ----------------------------------------------------------

    def connect(self) -> None:
        host, port = self._broker
        self._client.connect(host, port, keepalive=30)
        self._client.loop_start()
        for _ in range(50):
            if self._connected:
                return
            time.sleep(0.1)
        raise ConnectionError(f"Broker {host}:{port} did not confirm")

    def _on_connect(self, client, userdata, flags, rc) -> None:
        if rc != 0:
            logger.error("Broker refused connection, rc=%d", rc)
            return
        client.subscribe([(TOPIC_NODE_VIDEO, 0), (TOPIC_NODE_TELEMETRY, 0),
                          (TOPIC_NODE_EVENT, 1)])
        self._connected = True
        logger.info("Console connected to %s:%d", *self._broker)

    def _on_message(self, client, userdata, msg: mqtt.MQTTMessage) -> None:
        """Network thread: decode and stash only. Rendering happens on main."""
        try:
            if msg.topic == TOPIC_NODE_VIDEO:
                frame = decode_jpeg(msg.payload)
                if frame is not None:
                    try:
                        self._frames.put_nowait(frame)
                    except queue.Full:
                        pass  # newest-frame-wins; a stale frame is worthless
                return

            body = json.loads(msg.payload.decode("utf-8"))
            if msg.topic == TOPIC_NODE_TELEMETRY:
                self._telemetry = body
                self._state = body.get("state", self._state)
            elif msg.topic == TOPIC_NODE_EVENT:
                self._events.append(body)
                self._events[:] = self._events[-14:]
                self._last_event_at = time.monotonic()
                if body.get("event") == "state_transition":
                    self._state = body.get("to", self._state)
                elif body.get("event") == "firing_solution":
                    self._telemetry.update(body)
        except (ValueError, UnicodeDecodeError) as exc:
            # Console input is data, not instructions. Malformed is discarded.
            logger.debug("Discarding malformed message on %s: %s", msg.topic, exc)

    # ---- operator actions ----------------------------------------------

    def authorise(self, authorise: bool) -> None:
        """Publish the operator decision, bound to the active target."""
        target_id = self._telemetry.get("target_id")
        if not target_id:
            logger.warning("No active target; nothing to authorise")
            return
        nonce = uuid.uuid4().hex[:16]
        self._authorised_nonces.add(nonce)
        payload = {
            "auth": bool(authorise),
            "target_id": target_id,
            "token": self._auth_token or "",
            "nonce": nonce,
        }
        self._client.publish(TOPIC_OPERATOR_AUTH, json.dumps(payload), qos=1)
        logger.info("%s %s", "AUTHORISED" if authorise else "DENIED", target_id)

    def task_asset(self, asset: int) -> None:
        """Task simulated asset `asset` (1..3). The bridge picks the threat.

        Allowed in any node state: it labels a simulated asset on the map and
        cannot reach the real node or its effector.

        Thread: the console's main (render) thread.
        """
        payload = {"asset": asset, "token": self._auth_token or ""}
        self._client.publish(TOPIC_OPERATOR_TASK, json.dumps(payload), qos=1)
        logger.info("TASK simulated asset %d", asset)

    def send_test_cue(self) -> None:
        """Publish a synthetic radar cue at boresight."""
        payload = {"azimuth": 0.0, "elevation": 0.0,
                   "target_id": f"TRK-{uuid.uuid4().hex[:4].upper()}"}
        self._client.publish(TOPIC_SLEW_TO_CUE, json.dumps(payload), qos=1)
        logger.info("Test cue sent: %s", payload["target_id"])

    # ---- rendering -----------------------------------------------------

    def _current_frame(self) -> Optional[np.ndarray]:
        while True:
            try:
                self._last_frame = self._frames.get_nowait()
            except queue.Empty:
                break
        return self._last_frame

    def render(self) -> np.ndarray:
        canvas = np.full((PANEL_H, PANEL_W, 3), _BG, dtype=np.uint8)
        colour = _STATE_COLOUR.get(self._state, _WHITE)

        # Video pane.
        frame = self._current_frame()
        if frame is not None:
            scaled = cv2.resize(frame, (640, 360))
            canvas[60:420, 20:660] = scaled
        else:
            cv2.putText(canvas, "AWAITING VIDEO LINK", (150, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, _DIM, 2)
        cv2.rectangle(canvas, (20, 60), (660, 420), colour, 2)

        # State banner.
        cv2.rectangle(canvas, (0, 0), (PANEL_W, 46), colour, -1)
        cv2.putText(canvas, f"KILLSWITCH  //  {self._state}", (20, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (10, 10, 10), 2)
        link = "LINK OK" if self._connected else "NO LINK"
        cv2.putText(canvas, link, (PANEL_W - 130, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (10, 10, 10), 2)

        # Telemetry panel.
        y = 90
        cv2.putText(canvas, "TELEMETRY", (690, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, _WHITE, 1)
        for label, key, fmt in (
            ("target", "target_id", "{}"),
            ("track", "track_id", "#{}"),
            ("error", "error_px", "{} px"),
            ("hold", "hold_s", "{} s"),
            ("lead", "lead_ms", "{} ms"),
            ("speed", "target_speed_px_s", "{} px/s"),
        ):
            value = self._telemetry.get(key)
            text = fmt.format(value) if value is not None else "-"
            cv2.putText(canvas, f"{label:<8}{text}", (690, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, _WHITE if value else _DIM, 1)
            y += 26

        if self._telemetry.get("aimpoint_downgraded"):
            cv2.putText(canvas, "aimpoint gated ->", (690, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, _AMBER, 1)
            cv2.putText(canvas, "centre-of-mass", (690, y + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, _AMBER, 1)

        # Event log.
        cv2.putText(canvas, "EVENTS", (690, 320),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, _WHITE, 1)
        for i, event in enumerate(self._events[-7:]):
            name = str(event.get("event", "?"))[:16]
            if name == "state_transition":
                detail = f"{event.get('from', '?')[:4]}>{event.get('to', '?')[:9]}"
            else:
                detail = str(event.get("reason", ""))[:16]
            cv2.putText(canvas, f"{name[:15]:<16}{detail}", (690, 344 + i * 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, _DIM, 1)

        # The authorisation gate.
        pending = self._state == "OPERATOR_AUTH"
        cv2.rectangle(canvas, (20, 440), (660, 520),
                      _AMBER if pending else (50, 50, 56), -1 if pending else 1)
        if pending:
            cv2.putText(canvas, "AUTHORISATION REQUIRED", (44, 476),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.85, (10, 10, 10), 2)
            cv2.putText(canvas, "[SPACE] ENGAGE      [D] DENY", (44, 505),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (10, 10, 10), 2)
        else:
            cv2.putText(canvas, "autonomous: find - track - hold", (44, 476),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, _DIM, 1)
            cv2.putText(canvas, "no engagement without operator authorisation",
                        (44, 503), cv2.FONT_HERSHEY_SIMPLEX, 0.5, _DIM, 1)

        cv2.putText(canvas, "[SPACE] authorise  [D] deny  [1-3] task sim asset  [C] cue  [Q] quit",
                    (20, 560), cv2.FONT_HERSHEY_SIMPLEX, 0.5, _DIM, 1)
        cv2.putText(canvas, "Proxy effector: KY-008 laser. Firmware enforces deadman + burn ceiling.",
                    (20, 588), cv2.FONT_HERSHEY_SIMPLEX, 0.45, _DIM, 1)
        return canvas

    def run(self) -> int:
        window = "Killswitch // Operator Console"
        cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
        try:
            while True:
                cv2.imshow(window, self.render())
                key = cv2.waitKey(30) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord(" "):
                    if self._state == "OPERATOR_AUTH":
                        self.authorise(True)
                    else:
                        logger.warning(
                            "SPACE ignored: node is in %s, not OPERATOR_AUTH", self._state
                        )
                elif key == ord("d"):
                    self.authorise(False)
                elif key == ord("c"):
                    self.send_test_cue()
                elif key in (ord("1"), ord("2"), ord("3")):
                    self.task_asset(key - ord("0"))
        finally:
            cv2.destroyAllWindows()
            self._client.loop_stop()
            self._client.disconnect()
        return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/bench.yaml")
    parser.add_argument("--broker", default=None, help="Override broker host")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-7s %(message)s",
                        datefmt="%H:%M:%S")

    with open(args.config, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    c2 = config["c2"]

    console = OperatorConsole(
        broker=args.broker or c2["broker_host"],
        port=c2["broker_port"],
        auth_token=c2.get("auth_token"),
    )
    try:
        console.connect()
    except (ConnectionError, OSError) as exc:
        print(f"Could not reach the broker: {exc}")
        from helper.comms.mqtt_client import broker_start_hint
        print(f"Is mosquitto running?  {broker_start_hint()}")
        return 1
    return console.run()


if __name__ == "__main__":
    sys.exit(main())
