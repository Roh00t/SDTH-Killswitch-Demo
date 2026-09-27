"""Bench laser test: one key turns the KY-008 on, the same key turns it off.

    python -m tools.laser_test                 # port auto-detected
    python -m tools.laser_test --port COM3

    L        laser on (arms first); L again turns it off
    O/SPACE  laser off
    Q/ESC    off, disarm, park, quit

This is a BENCH tool, not part of the live system. It talks straight to the
firmware over the USB-UART bridge, so it needs COM3 and cannot run while
main.py holds it. In a live run the laser fires one way only: a locked
track, HOLD, then SPACE in the operator console. guardrails.md forbids any
remote `laser:1`, so there is deliberately no laser key in the console.

Every firmware interlock still applies: arming and firing are two separate
checksummed commands, the firmware cuts any burn at 2.0 s on its own, and it
cuts the beam 250 ms after this tool stops sending heartbeats (a crash, a
closed window, a pulled cable).
"""
from __future__ import annotations

import argparse
import logging
import sys
import time
from typing import Callable, Optional

from helper.hardware.actuator import ActuatorDriver, ActuatorError, SerialActuator, resolve_port
from helper.hardware.protocol import BAUD_RATE, MAX_BURN_MS

# How long to wait for a status frame to confirm the laser really changed.
CONFIRM_TIMEOUT_S: float = 0.5
POLL_INTERVAL_S: float = 0.05


class LaserBench:
    """Toggle state for one laser on one actuator.

    Thread: owned by the main thread. The actuator's own reader and heartbeat
    threads run underneath it.
    """

    def __init__(self, actuator: ActuatorDriver, say: Callable[[str], None] = print) -> None:
        """Args:
            actuator: A connected driver (serial on the rig, mock in tests).
            say: Where status lines go.
        """
        self._actuator = actuator
        self._say = say
        self._on = False

    @property
    def is_on(self) -> bool:
        """What this tool last commanded and saw confirmed."""
        return self._on

    def toggle(self) -> None:
        """Laser on if it is off, off if it is on."""
        if self._on:
            self.off()
        else:
            self.on()

    def on(self) -> None:
        """Arm, fire, and confirm the firmware reports the laser lit.

        Raises:
            ActuatorError: The link failed.
        """
        self._actuator.arm()
        self._actuator.set_effector(True)
        deadline = time.monotonic() + CONFIRM_TIMEOUT_S
        while time.monotonic() < deadline:
            status = self._actuator.last_status()
            if status is not None and status.laser_on:
                self._on = True
                self._say(f"  LASER ON   (firmware cuts it at {MAX_BURN_MS / 1000:.1f} s; "
                          "L or O turns it off sooner)")
                return
            time.sleep(POLL_INTERVAL_S)
        self._actuator.set_effector(False)
        self._say("  firmware did not report the laser on; sent off again. "
                  "Check the log above for an ERR line.")

    def off(self) -> None:
        """De-energise and confirm via a status frame. Never raises."""
        try:
            self._actuator.set_effector(False)
        except ActuatorError as exc:
            self._say(f"  off command failed ({exc}); the firmware deadman cuts it in 250 ms")
        confirmed = self._actuator.confirm_effector_off(timeout=CONFIRM_TIMEOUT_S)
        self._on = False
        self._say("  laser off" if confirmed else
                  "  LASER OFF NOT CONFIRMED: treat it as live until the firmware deadman")

    def poll(self) -> None:
        """Notice a burn the firmware ended by itself (the 2 s ceiling)."""
        if not self._on:
            return
        status = self._actuator.last_status()
        if status is not None and not status.laser_on:
            self._on = False
            self._say("  laser off (firmware 2.0 s burn ceiling)")

    def shutdown(self) -> None:
        """Off, then disarm. Called on every exit path."""
        if self._on:
            self.off()
        try:
            self._actuator.disarm()
        except ActuatorError as exc:
            self._say(f"  disarm failed ({exc}); close() still e-stops")


def _read_key() -> Optional[str]:
    """One pending keypress, lower-cased, or None. Windows console only."""
    import msvcrt

    if not msvcrt.kbhit():
        return None
    key = msvcrt.getwch()
    if key in ("\x00", "\xe0"):   # arrow/function key prefix: swallow the pair
        msvcrt.getwch()
        return None
    return key.lower()


def run(port: str) -> int:
    """Connect, confirm the backstop, then serve keys until Q."""
    print("\nThe laser will light when you press L. Before going on:")
    print("  - a MATTE backstop is in the beam path (no mirrors, glass or screens)")
    print("  - nobody is in or near the beam path")
    if input("Type YES to continue: ").strip() != "YES":
        print("Not confirmed. Nothing was sent to the board.")
        return 1

    actuator = SerialActuator(port, baud=BAUD_RATE)
    bench = LaserBench(actuator)
    try:
        actuator.connect()
        print("\n  L = laser on/off    O or SPACE = off    Q = quit\n")
        if sys.platform == "win32":
            while True:
                key = _read_key()
                if key == "l":
                    bench.toggle()
                elif key in ("o", " "):
                    bench.off()
                elif key in ("q", "\x1b", "\x03"):
                    break
                bench.poll()
                time.sleep(POLL_INTERVAL_S)
        else:   # no single-key read: fall back to Enter-terminated letters
            while True:
                key = input("> ").strip().lower()
                bench.poll()
                if key == "l":
                    bench.toggle()
                elif key == "o":
                    bench.off()
                elif key == "q":
                    break
        return 0
    except KeyboardInterrupt:
        return 0
    except ActuatorError as exc:
        print(f"\nACTUATOR ERROR: {exc}")
        return 1
    finally:
        bench.shutdown()
        actuator.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="auto", help="Serial port, or auto (default)")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    try:
        port = resolve_port(args.port)
    except ActuatorError as exc:
        print(f"\n{exc}\n")
        return 1
    return run(port)


if __name__ == "__main__":
    sys.exit(main())
